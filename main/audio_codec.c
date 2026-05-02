#include "audio_codec.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "audio_codec";

#define AUDIO_CODEC_I2C_SDA             21
#define AUDIO_CODEC_I2C_SCL             22
#define AUDIO_CODEC_I2C_HZ              100000
#define AUDIO_CODEC_I2C_TIMEOUT_MS      1000
#define AUDIO_CODEC_CTRL_MCU_ADDR       0x33
#define AUDIO_CODEC_ES8388_ADDR         0x10

#define AUDIO_CODEC_I2S_MCLK            0
#define AUDIO_CODEC_I2S_BCLK            13
#define AUDIO_CODEC_I2S_LRCK            12
#define AUDIO_CODEC_I2S_DOUT            15
#define AUDIO_CODEC_I2S_DIN             34

#define AUDIO_CODEC_SAMPLE_RATE_HZ      8000
#define AUDIO_CODEC_I2S_BITS            I2S_DATA_BIT_WIDTH_16BIT
#define AUDIO_CODEC_I2S_SLOT_MODE       I2S_SLOT_MODE_MONO
#define AUDIO_CODEC_I2S_RX_SLOT_MODE    I2S_SLOT_MODE_STEREO
#define AUDIO_CODEC_PRELOAD_SAMPLES     256
#define AUDIO_CODEC_TEST_TONE_HZ        1000
#define AUDIO_CODEC_TEST_TONE_AMPLITUDE 5000
#define AUDIO_CODEC_TEST_TONE_CHUNK_MS  20

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_ctrl_mcu;
static i2c_master_dev_handle_t s_es8388;
static i2s_chan_handle_t s_tx_chan;
static i2s_chan_handle_t s_rx_chan;
static bool s_playback_enabled;
static bool s_capture_enabled;

static esp_err_t es8388_write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t payload[] = {reg, value};
    return i2c_master_transmit(s_es8388, payload, sizeof(payload), AUDIO_CODEC_I2C_TIMEOUT_MS);
}

static esp_err_t ctrl_mcu_write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t payload[] = {reg, value};
    return i2c_master_transmit(s_ctrl_mcu, payload, sizeof(payload), AUDIO_CODEC_I2C_TIMEOUT_MS);
}

static esp_err_t ctrl_mcu_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(s_ctrl_mcu, &reg, sizeof(reg), value, 1, AUDIO_CODEC_I2C_TIMEOUT_MS);
}

static esp_err_t module_audio_configure_headset(void)
{
    uint8_t hp_status = 0;

    /* Module Audio's STM32 controls TRRS standard switching and mic path enable.
     * Force the common smartphone headset settings before using the ES8388 codec. */
    ESP_RETURN_ON_ERROR(ctrl_mcu_write_reg(0x10, 0x00), TAG, "ctrl_mcu set headphone mode failed");
    ESP_RETURN_ON_ERROR(ctrl_mcu_write_reg(0x00, 0x01), TAG, "ctrl_mcu set mic status failed");
    ESP_RETURN_ON_ERROR(ctrl_mcu_read_reg(0x20, &hp_status), TAG, "ctrl_mcu read headphone insert failed");

    ESP_LOGI(
        TAG,
        "Module Audio control MCU ready: HP mode=OMTP/NATIONAL, MIC=open, hp_status_raw=%u",
        (unsigned)hp_status);
    return ESP_OK;
}

static esp_err_t es8388_configure_playback(void)
{
    uint8_t adc_fs_ratio = 0x02;
    uint8_t dac_fs_ratio = 0x02;

    switch (AUDIO_CODEC_SAMPLE_RATE_HZ) {
    case 8000:
        adc_fs_ratio = 0x0A;
        dac_fs_ratio = 0x0A;
        break;
    case 16000:
        adc_fs_ratio = 0x06;
        dac_fs_ratio = 0x06;
        break;
    case 44100:
    case 48000:
        adc_fs_ratio = 0x02;
        dac_fs_ratio = 0x02;
        break;
    default:
        ESP_LOGW(TAG, "Unsupported sample rate %d for explicit ES8388 ratio setup, using 256fs defaults", AUDIO_CODEC_SAMPLE_RATE_HZ);
        break;
    }

    static const struct {
        uint8_t reg;
        uint8_t value;
    } init_seq_template[] = {
        {0x08, 0x00}, /* slave mode */
        {0x02, 0xFF}, /* power down DEM/STM during init */
        {0x2B, 0x80}, /* same LRCK */
        {0x00, 0x05}, /* play + record mode */
        {0x01, 0x40}, /* power up analog + ibias */
        {0x03, 0x00}, /* ADC power */
        {0x0A, 0x50}, /* TRRS headset mic path: LINPUT2/RINPUT2 */
        {0x0B, 0x80}, /* input2 path requires ADC Control 3 high-bit set in M5 API */
        {0x09, 0x88}, /* mic PGA gain +24 dB max valid value */
        {0x0C, 0x2C}, /* I2S, 16-bit */
        {0x0D, 0x00}, /* ADC fs ratio, filled below */
        {0x0F, 0x28},
        {0x10, 0x00},
        {0x11, 0x00},
        {0x12, 0xEA},
        {0x13, 0xC0},
        {0x14, 0x12},
        {0x15, 0x06},
        {0x16, 0xC3},
        {0x04, 0x3F}, /* official init: power DAC and enable all outputs first */
        {0x17, 0x18}, /* I2S, 16-bit */
        {0x18, 0x00}, /* DAC fs ratio, filled below */
        {0x19, 0x00}, /* unmute */
        {0x1A, 0x05}, /* DAC digital volume left */
        {0x1B, 0x05}, /* DAC digital volume right */
        {0x26, 0x00}, /* mixer source select */
        {0x27, 0xD0}, /* left DAC -> left mixer */
        {0x28, 0x38},
        {0x29, 0x38},
        {0x2A, 0xD0}, /* right DAC -> right mixer */
        {0x2B, 0x80},
        {0x2E, 0x1A}, /* DAC volume ~= 80 from official helper */
        {0x2F, 0x1A},
        {0x30, 0x00}, /* OUT2 volume stays at 0 dB/default path */
        {0x31, 0x00},
        {0x04, 0x30}, /* official playback example selects DAC_OUTPUT_OUT1 */
        {0x02, 0x00}, /* power up DEM/STM */
    };
    struct {
        uint8_t reg;
        uint8_t value;
    } init_seq[sizeof(init_seq_template) / sizeof(init_seq_template[0])];

    memcpy(init_seq, init_seq_template, sizeof(init_seq_template));
    init_seq[10].value = adc_fs_ratio;
    init_seq[20].value = dac_fs_ratio;

    vTaskDelay(pdMS_TO_TICKS(20));
    for (size_t i = 0; i < sizeof(init_seq) / sizeof(init_seq[0]); ++i) {
        ESP_RETURN_ON_ERROR(es8388_write_reg(init_seq[i].reg, init_seq[i].value), TAG, "ES8388 write failed at reg 0x%02x", init_seq[i].reg);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_OK;
}

esp_err_t audio_codec_init(void)
{
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = AUDIO_CODEC_I2C_SDA,
        .scl_io_num = AUDIO_CODEC_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_device_config_t es8388_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AUDIO_CODEC_ES8388_ADDR,
        .scl_speed_hz = AUDIO_CODEC_I2C_HZ,
    };
    i2c_device_config_t ctrl_mcu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AUDIO_CODEC_CTRL_MCU_ADDR,
        .scl_speed_hz = AUDIO_CODEC_I2C_HZ,
    };
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_CODEC_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(AUDIO_CODEC_I2S_BITS, AUDIO_CODEC_I2S_SLOT_MODE),
        .gpio_cfg = {
            .mclk = AUDIO_CODEC_I2S_MCLK,
            .bclk = AUDIO_CODEC_I2S_BCLK,
            .ws = AUDIO_CODEC_I2S_LRCK,
            .dout = AUDIO_CODEC_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_CODEC_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(AUDIO_CODEC_I2S_BITS, AUDIO_CODEC_I2S_RX_SLOT_MODE),
        .gpio_cfg = {
            .mclk = AUDIO_CODEC_I2S_MCLK,
            .bclk = AUDIO_CODEC_I2S_BCLK,
            .ws = AUDIO_CODEC_I2S_LRCK,
            .dout = I2S_GPIO_UNUSED,
            .din = AUDIO_CODEC_I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    tx_chan_cfg.auto_clear = true;
#if CONFIG_IDF_TARGET_ESP32
    tx_std_cfg.slot_cfg.msb_right = false;
    rx_std_cfg.slot_cfg.msb_right = false;
#endif

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_bus_cfg, &s_i2c_bus), TAG, "i2c_new_master_bus failed");
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &ctrl_mcu_cfg, &s_ctrl_mcu), TAG, "i2c_master_bus_add_device ctrl_mcu failed");
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &es8388_cfg, &s_es8388), TAG, "i2c_master_bus_add_device failed");
    ESP_RETURN_ON_ERROR(module_audio_configure_headset(), TAG, "module_audio_configure_headset failed");
    ESP_RETURN_ON_ERROR(es8388_configure_playback(), TAG, "es8388_configure_playback failed");

    ESP_RETURN_ON_ERROR(i2s_new_channel(&tx_chan_cfg, &s_tx_chan, &s_rx_chan), TAG, "i2s_new_channel failed");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx_chan, &tx_std_cfg), TAG, "i2s_channel_init_std_mode failed");
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx_chan, &rx_std_cfg), TAG, "i2s_channel_init_std_mode rx failed");

    ESP_LOGI(
        TAG,
        "Phase 2 codec ready: ES8388 over I2C SDA=%d SCL=%d, I2S MCLK=%d BCLK=%d LRCK=%d DOUT=%d DIN=%d @ %d Hz",
        AUDIO_CODEC_I2C_SDA,
        AUDIO_CODEC_I2C_SCL,
        AUDIO_CODEC_I2S_MCLK,
        AUDIO_CODEC_I2S_BCLK,
        AUDIO_CODEC_I2S_LRCK,
        AUDIO_CODEC_I2S_DOUT,
        AUDIO_CODEC_I2S_DIN,
        AUDIO_CODEC_SAMPLE_RATE_HZ);
    return ESP_OK;
}

esp_err_t audio_codec_start_playback(void)
{
    int16_t silence[AUDIO_CODEC_PRELOAD_SAMPLES * 2] = {0};
    size_t bytes_loaded = 0;

    if (s_playback_enabled) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(
        i2s_channel_preload_data(s_tx_chan, silence, sizeof(silence), &bytes_loaded),
        TAG,
        "i2s_channel_preload_data failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx_chan), TAG, "i2s_channel_enable failed");
    s_playback_enabled = true;
    ESP_LOGI(TAG, "Playback path enabled (preloaded %u bytes of silence)", (unsigned)bytes_loaded);
    return ESP_OK;
}

esp_err_t audio_codec_stop_playback(void)
{
    if (!s_playback_enabled) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2s_channel_disable(s_tx_chan), TAG, "i2s_channel_disable failed");
    s_playback_enabled = false;
    ESP_LOGI(TAG, "Playback path disabled");
    return ESP_OK;
}

esp_err_t audio_codec_start_capture(void)
{
    if (s_capture_enabled) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "i2s_channel_enable rx failed");
    s_capture_enabled = true;
    ESP_LOGI(TAG, "Capture path enabled");
    return ESP_OK;
}

esp_err_t audio_codec_stop_capture(void)
{
    if (!s_capture_enabled) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2s_channel_disable(s_rx_chan), TAG, "i2s_channel_disable rx failed");
    s_capture_enabled = false;
    ESP_LOGI(TAG, "Capture path disabled");
    return ESP_OK;
}

esp_err_t audio_codec_read_capture(void *data, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    if (!s_capture_enabled) {
        if (bytes_read) {
            *bytes_read = 0;
        }
        return ESP_ERR_INVALID_STATE;
    }

    return i2s_channel_read(s_rx_chan, (void *)data, len, bytes_read, timeout_ms);
}

esp_err_t audio_codec_write_playback(const void *data, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    if (!s_playback_enabled) {
        if (bytes_written) {
            *bytes_written = 0;
        }
        return ESP_ERR_INVALID_STATE;
    }

    return i2s_channel_write(s_tx_chan, data, len, bytes_written, timeout_ms);
}

esp_err_t audio_codec_play_test_tone(uint32_t duration_ms)
{
    if (duration_ms == 0) {
        return ESP_OK;
    }

    bool started_here = false;
    if (!s_playback_enabled) {
        ESP_RETURN_ON_ERROR(audio_codec_start_playback(), TAG, "audio_codec_start_playback failed");
        started_here = true;
    }

    const uint32_t chunk_samples = (AUDIO_CODEC_SAMPLE_RATE_HZ * AUDIO_CODEC_TEST_TONE_CHUNK_MS) / 1000;
    int16_t mono_buf[chunk_samples];
    const double omega = (2.0 * M_PI * (double)AUDIO_CODEC_TEST_TONE_HZ) / (double)AUDIO_CODEC_SAMPLE_RATE_HZ;
    uint32_t total_samples = (AUDIO_CODEC_SAMPLE_RATE_HZ * duration_ms) / 1000;
    uint32_t generated = 0;

    while (generated < total_samples) {
        uint32_t mono_count = total_samples - generated;
        if (mono_count > chunk_samples) {
            mono_count = chunk_samples;
        }

        for (uint32_t i = 0; i < mono_count; ++i) {
            double phase = (double)(generated + i) * omega;
            int16_t sample = (int16_t)(sin(phase) * AUDIO_CODEC_TEST_TONE_AMPLITUDE);
            mono_buf[i] = sample;
        }

        size_t bytes_written = 0;
        ESP_RETURN_ON_ERROR(
            audio_codec_write_playback(mono_buf, mono_count * sizeof(int16_t), &bytes_written, portMAX_DELAY),
            TAG,
            "audio_codec_write_playback test tone failed");
        generated += mono_count;
    }

    ESP_LOGI(TAG, "Playback test tone generated: %u ms @ %u Hz", (unsigned)duration_ms, AUDIO_CODEC_TEST_TONE_HZ);

    if (started_here) {
        ESP_RETURN_ON_ERROR(audio_codec_stop_playback(), TAG, "audio_codec_stop_playback failed");
    }

    return ESP_OK;
}
