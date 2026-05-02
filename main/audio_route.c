#include "audio_route.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "audio_codec.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

static const char *TAG = "audio_route";

#define AUDIO_ROUTE_RX_RINGBUF_BYTES    65536
#define AUDIO_ROUTE_TX_RINGBUF_BYTES    32768
#define AUDIO_ROUTE_WRITE_TASK_STACK    4096
#define AUDIO_ROUTE_CAPTURE_TASK_STACK  4096
#define AUDIO_ROUTE_WRITE_TASK_PRIO     10
#define AUDIO_ROUTE_CAPTURE_TASK_PRIO   4
#define AUDIO_ROUTE_PLAYBACK_CHUNK_SAMPLES 240
#define AUDIO_ROUTE_CAPTURE_CHUNK_SAMPLES  80
#define AUDIO_ROUTE_ENABLE_SIDETONE_DIAG  0
#define AUDIO_ROUTE_CAPTURE_TIMEOUT_MS    5

static RingbufHandle_t s_rx_ringbuf;
static RingbufHandle_t s_tx_ringbuf;
static TaskHandle_t s_playback_task;
static TaskHandle_t s_capture_task;
static bool s_playback_active;
static bool s_capture_active;
static uint32_t s_capture_chunk_count;
static uint32_t s_capture_low_energy_count;
static bool s_capture_slot_locked;
static bool s_capture_use_left = true;
static uint32_t s_capture_probe_chunks;
static int64_t s_capture_probe_left_sum;
static int64_t s_capture_probe_right_sum;

static void playback_task(void *arg)
{
    int16_t mono_buf[AUDIO_ROUTE_PLAYBACK_CHUNK_SAMPLES];

    while (true) {
        size_t item_size = 0;
        uint8_t *item = xRingbufferReceive(s_rx_ringbuf, &item_size, portMAX_DELAY);
        if (item == NULL) {
            continue;
        }

        if (!s_playback_active) {
            vRingbufferReturnItem(s_rx_ringbuf, item);
            continue;
        }

        size_t consumed = 0;
        while ((consumed + sizeof(int16_t)) <= item_size) {
            size_t mono_samples = (item_size - consumed) / sizeof(int16_t);
            if (mono_samples > AUDIO_ROUTE_PLAYBACK_CHUNK_SAMPLES) {
                mono_samples = AUDIO_ROUTE_PLAYBACK_CHUNK_SAMPLES;
            }

            const int16_t *mono = (const int16_t *)(item + consumed);
            for (size_t i = 0; i < mono_samples; ++i) {
                int16_t sample = (int16_t)(mono[i] / 2);
                mono_buf[i] = sample;
            }

            size_t chunk_bytes = mono_samples * sizeof(int16_t);
            size_t written_total = 0;
            while (written_total < chunk_bytes) {
                size_t bytes_written = 0;
                esp_err_t err = audio_codec_write_playback(
                    ((const uint8_t *)mono_buf) + written_total,
                    chunk_bytes - written_total,
                    &bytes_written,
                    portMAX_DELAY);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "audio_codec_write_playback failed: %s", esp_err_to_name(err));
                    written_total = 0;
                    break;
                }
                if (bytes_written == 0) {
                    ESP_LOGW(TAG, "audio_codec_write_playback wrote 0 bytes");
                    written_total = 0;
                    break;
                }
                written_total += bytes_written;
            }
            if (written_total == 0) {
                break;
            }

            consumed += written_total;
        }

        vRingbufferReturnItem(s_rx_ringbuf, item);
    }
}

static void capture_task(void *arg)
{
    int16_t stereo_buf[AUDIO_ROUTE_CAPTURE_CHUNK_SAMPLES * 2];
    int16_t mono_buf[AUDIO_ROUTE_CAPTURE_CHUNK_SAMPLES];

    while (true) {
        if (!s_capture_active) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t bytes_read = 0;
        esp_err_t err = audio_codec_read_capture(stereo_buf, sizeof(stereo_buf), &bytes_read, pdMS_TO_TICKS(AUDIO_ROUTE_CAPTURE_TIMEOUT_MS));
        if (err != ESP_OK) {
            if (err != ESP_ERR_INVALID_STATE && err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "audio_codec_read_capture failed: %s", esp_err_to_name(err));
            }
            if (bytes_read == 0) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
        }
        if (bytes_read == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        size_t stereo_samples = bytes_read / sizeof(int16_t);
        size_t frames = stereo_samples / 2U;
        if (frames > AUDIO_ROUTE_CAPTURE_CHUNK_SAMPLES) {
            frames = AUDIO_ROUTE_CAPTURE_CHUNK_SAMPLES;
        }
        if (frames == 0) {
            continue;
        }

        int32_t left_peak = 0;
        int32_t right_peak = 0;
        int64_t left_sum_abs = 0;
        int64_t right_sum_abs = 0;
        for (size_t i = 0; i < frames; ++i) {
            int16_t left = stereo_buf[i * 2U];
            int16_t right = stereo_buf[(i * 2U) + 1U];
            int32_t left_abs = left < 0 ? -(int32_t)left : (int32_t)left;
            int32_t right_abs = right < 0 ? -(int32_t)right : (int32_t)right;
            if (left_abs > left_peak) {
                left_peak = left_abs;
            }
            if (right_abs > right_peak) {
                right_peak = right_abs;
            }
            left_sum_abs += left_abs;
            right_sum_abs += right_abs;
        }

        bool use_left = s_capture_use_left;
        if (!s_capture_slot_locked) {
            s_capture_probe_chunks++;
            s_capture_probe_left_sum += left_sum_abs;
            s_capture_probe_right_sum += right_sum_abs;

            if (s_capture_probe_chunks >= 50U) {
                use_left = s_capture_probe_left_sum >= s_capture_probe_right_sum;
                s_capture_use_left = use_left;
                s_capture_slot_locked = true;
                ESP_LOGI(
                    TAG,
                    "Mic capture slot locked: slot=%s left_sum=%" PRIi64 " right_sum=%" PRIi64 " probe_chunks=%" PRIu32,
                    use_left ? "L" : "R",
                    s_capture_probe_left_sum,
                    s_capture_probe_right_sum,
                    s_capture_probe_chunks);
            } else {
                use_left = left_sum_abs >= right_sum_abs;
                s_capture_use_left = use_left;
            }
        }
        for (size_t i = 0; i < frames; ++i) {
            mono_buf[i] = stereo_buf[(i * 2U) + (use_left ? 0U : 1U)];
        }

        size_t mono_bytes = frames * sizeof(int16_t);
        s_capture_chunk_count++;
        int32_t peak = 0;
        int64_t sum_abs = 0;
        for (size_t i = 0; i < frames; ++i) {
            int32_t v = mono_buf[i];
            int32_t a = v < 0 ? -v : v;
            if (a > peak) {
                peak = a;
            }
            sum_abs += a;
        }
        int64_t avg_abs = frames ? (sum_abs / (int64_t)frames) : 0;
        if ((s_capture_chunk_count % 100U) == 0U) {
            ESP_LOGI(
                TAG,
                "Mic capture stats: chunks=%" PRIu32 " peak=%" PRId32 " avg_abs=%" PRIi64 " samples=%u slot=%s left_peak=%" PRId32 " left_avg=%" PRIi64 " right_peak=%" PRId32 " right_avg=%" PRIi64,
                s_capture_chunk_count,
                peak,
                avg_abs,
                (unsigned)frames,
                use_left ? "L" : "R",
                left_peak,
                frames ? (left_sum_abs / (int64_t)frames) : 0,
                right_peak,
                frames ? (right_sum_abs / (int64_t)frames) : 0);
        }

        if ((avg_abs < 32) && (++s_capture_low_energy_count % 200U) == 0U) {
            ESP_LOGI(TAG, "Mic capture remains near-silent: low_energy_chunks=%" PRIu32, s_capture_low_energy_count);
        }

#if AUDIO_ROUTE_ENABLE_SIDETONE_DIAG
        if (s_playback_active && s_rx_ringbuf != NULL) {
            int16_t sidetone_buf[AUDIO_ROUTE_CAPTURE_CHUNK_SAMPLES];
            for (size_t i = 0; i < frames; ++i) {
                sidetone_buf[i] = (int16_t)(mono_buf[i] / 8);
            }
            BaseType_t playback_ok = xRingbufferSend(
                s_rx_ringbuf,
                sidetone_buf,
                mono_bytes,
                0);
            if (playback_ok != pdTRUE && (s_capture_chunk_count % 200U) == 0U) {
                ESP_LOGW(TAG, "Dropping sidetone audio: playback ringbuffer full (%u bytes)", (unsigned)mono_bytes);
            }
        }
#endif

        BaseType_t ok = xRingbufferSend(s_tx_ringbuf, mono_buf, mono_bytes, 0);
        if (ok != pdTRUE) {
            ESP_LOGW(TAG, "Dropping TX audio: ringbuffer full (%u bytes)", (unsigned)mono_bytes);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t audio_route_init(void)
{
    s_rx_ringbuf = xRingbufferCreate(AUDIO_ROUTE_RX_RINGBUF_BYTES, RINGBUF_TYPE_BYTEBUF);
    ESP_RETURN_ON_FALSE(s_rx_ringbuf != NULL, ESP_ERR_NO_MEM, TAG, "xRingbufferCreate failed");
    s_tx_ringbuf = xRingbufferCreate(AUDIO_ROUTE_TX_RINGBUF_BYTES, RINGBUF_TYPE_BYTEBUF);
    ESP_RETURN_ON_FALSE(s_tx_ringbuf != NULL, ESP_ERR_NO_MEM, TAG, "xRingbufferCreate tx failed");

    BaseType_t task_ok = xTaskCreate(
        playback_task,
        "audio_playback",
        AUDIO_ROUTE_WRITE_TASK_STACK,
        NULL,
        AUDIO_ROUTE_WRITE_TASK_PRIO,
        &s_playback_task);
    ESP_RETURN_ON_FALSE(task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "xTaskCreate failed");
    task_ok = xTaskCreatePinnedToCore(
        capture_task,
        "audio_capture",
        AUDIO_ROUTE_CAPTURE_TASK_STACK,
        NULL,
        AUDIO_ROUTE_CAPTURE_TASK_PRIO,
        &s_capture_task,
        0);
    ESP_RETURN_ON_FALSE(task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "xTaskCreate capture failed");

    ESP_LOGI(TAG, "Phase 2 route ready: HFP RX ringbuffer=%d bytes, TX ringbuffer=%d bytes", AUDIO_ROUTE_RX_RINGBUF_BYTES, AUDIO_ROUTE_TX_RINGBUF_BYTES);
    return ESP_OK;
}

esp_err_t audio_route_start_playback(void)
{
    ESP_RETURN_ON_ERROR(audio_codec_start_playback(), TAG, "audio_codec_start_playback failed");
    s_playback_active = true;
    ESP_LOGI(TAG, "HFP RX -> I2S playback started");
    return ESP_OK;
}

esp_err_t audio_route_stop_playback(void)
{
    s_playback_active = false;
    if (s_rx_ringbuf) {
        vRingbufferReset(s_rx_ringbuf);
    }
    ESP_RETURN_ON_ERROR(audio_codec_stop_playback(), TAG, "audio_codec_stop_playback failed");
    ESP_LOGI(TAG, "HFP RX -> I2S playback stopped");
    return ESP_OK;
}

esp_err_t audio_route_start_capture(void)
{
    if (s_tx_ringbuf) {
        vRingbufferReset(s_tx_ringbuf);
    }
    s_capture_chunk_count = 0;
    s_capture_low_energy_count = 0;
    s_capture_slot_locked = false;
    s_capture_use_left = true;
    s_capture_probe_chunks = 0;
    s_capture_probe_left_sum = 0;
    s_capture_probe_right_sum = 0;
    ESP_RETURN_ON_ERROR(audio_codec_start_capture(), TAG, "audio_codec_start_capture failed");
    s_capture_active = true;
    ESP_LOGI(TAG, "Mic capture -> HFP TX started (sidetone diag=%d)", AUDIO_ROUTE_ENABLE_SIDETONE_DIAG);
    return ESP_OK;
}

esp_err_t audio_route_stop_capture(void)
{
    s_capture_active = false;
    if (s_tx_ringbuf) {
        vRingbufferReset(s_tx_ringbuf);
    }
    ESP_RETURN_ON_ERROR(audio_codec_stop_capture(), TAG, "audio_codec_stop_capture failed");
    ESP_LOGI(TAG, "Mic capture -> HFP TX stopped");
    return ESP_OK;
}

size_t audio_route_take_mic_pcm(int16_t *samples, size_t max_samples, uint32_t timeout_ms)
{
    if (!s_capture_active || s_tx_ringbuf == NULL || samples == NULL || max_samples == 0) {
        return 0;
    }

    size_t item_size = 0;
    uint8_t *item = xRingbufferReceiveUpTo(s_tx_ringbuf, &item_size, pdMS_TO_TICKS(timeout_ms), max_samples * sizeof(int16_t));
    if (item == NULL || item_size == 0) {
        return 0;
    }

    memcpy(samples, item, item_size);
    vRingbufferReturnItem(s_tx_ringbuf, item);
    return item_size / sizeof(int16_t);
}

void audio_route_handle_hfp_rx_pcm(const int16_t *samples, size_t sample_count)
{
    if (!s_playback_active || s_rx_ringbuf == NULL || samples == NULL || sample_count == 0) {
        return;
    }

    size_t len = sample_count * sizeof(int16_t);
    BaseType_t ok = xRingbufferSend(s_rx_ringbuf, samples, len, pdMS_TO_TICKS(5));
    if (ok != pdTRUE) {
        ESP_LOGW(TAG, "Dropping RX audio: ringbuffer full (%u bytes)", (unsigned)len);
    }
}
