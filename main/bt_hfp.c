#include "bt_hfp.h"

#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "audio_route.h"
#include "cvsd_codec.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_hf_client_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "bt_hfp";
static const char *DEVICE_NAME = "ESP-HEADSET";
#define HFP_RX_FORCE_BYTESWAP_PCM   1
#define HFP_TX_ENABLE_MIC_UPLINK    1
#define CVSD_FORCED_MODE            1
#if !HFP_RX_FORCE_BYTESWAP_PCM
static const cvsd_mode_t s_cvsd_modes[] = {
    {.lsb_first = true, .invert_symbol = false},
    {.lsb_first = false, .invert_symbol = false},
    {.lsb_first = true, .invert_symbol = true},
    {.lsb_first = false, .invert_symbol = true},
};
#endif
static bool s_audio_connected;
static bool s_data_cb_registered;
static esp_hf_sync_conn_hdl_t s_sync_conn_hdl;
static uint32_t s_rx_packet_count;
static uint32_t s_pcm_snapshot_count;
static cvsd_decoder_t s_cvsd_decoders[4];
static bool s_cvsd_decoders_ready;
static int s_cvsd_locked_mode = -1;
static uint32_t s_tx_packet_count;
static cvsd_encoder_t s_cvsd_encoder;
static bool s_cvsd_encoder_ready;
static uint32_t s_rx_voice_active_count;
static uint32_t s_rx_voice_silent_count;

static void log_cvsd_snapshot(const uint8_t *input, uint32_t input_len, const int16_t *samples, size_t sample_count, int mode_index)
{
    char raw_buf[3 * 12 + 1] = {0};
    char pcm_buf[8 * 12 + 1] = {0};
    size_t raw_count = input_len < 12 ? input_len : 12;
    size_t pcm_count = sample_count < 12 ? sample_count : 12;
    size_t off = 0;

    for (size_t i = 0; i < raw_count && off < sizeof(raw_buf); ++i) {
        off += (size_t)snprintf(raw_buf + off, sizeof(raw_buf) - off, "%02X%s", input[i], (i + 1U < raw_count) ? " " : "");
    }

    off = 0;
    for (size_t i = 0; i < pcm_count && off < sizeof(pcm_buf); ++i) {
        off += (size_t)snprintf(pcm_buf + off, sizeof(pcm_buf) - off, "%d%s", (int)samples[i], (i + 1U < pcm_count) ? " " : "");
    }

    ESP_LOGI(
        TAG,
        "CVSD snapshot: mode=%d raw[0..%u]=%s pcm[0..%u]=%s",
        mode_index,
        (unsigned)(raw_count ? (raw_count - 1U) : 0U),
        raw_buf,
        (unsigned)(pcm_count ? (pcm_count - 1U) : 0U),
        pcm_buf);
}

static void reset_cvsd_state(void)
{
    for (size_t i = 0; i < (sizeof(s_cvsd_decoders) / sizeof(s_cvsd_decoders[0])); ++i) {
        cvsd_decoder_init(&s_cvsd_decoders[i]);
    }
    s_cvsd_decoders_ready = true;
    s_cvsd_locked_mode = CVSD_FORCED_MODE;
    s_rx_packet_count = 0;
    s_tx_packet_count = 0;
    s_pcm_snapshot_count = 0;
    s_rx_voice_active_count = 0;
    s_rx_voice_silent_count = 0;
    cvsd_encoder_init(&s_cvsd_encoder);
    s_cvsd_encoder_ready = true;
}

static int64_t score_pcm_candidate(const int16_t *samples, size_t sample_count, int32_t *peak_out, int64_t *avg_abs_out)
{
    int32_t peak = 0;
    int64_t sum_abs = 0;
    int64_t sum_diff = 0;
    int64_t sum = 0;
    int32_t min_sample = INT32_MAX;
    int32_t max_sample = INT32_MIN;
    int32_t clipped = 0;
    int32_t zero_cross = 0;

    for (size_t i = 0; i < sample_count; ++i) {
        int32_t sample = samples[i];
        int32_t abs_sample = sample < 0 ? -sample : sample;
        if (abs_sample > peak) {
            peak = abs_sample;
        }
        if (sample < min_sample) {
            min_sample = sample;
        }
        if (sample > max_sample) {
            max_sample = sample;
        }
        if (abs_sample >= 30000) {
            clipped++;
        }
        sum_abs += abs_sample;
        sum += sample;
        if (i != 0) {
            int32_t diff = sample - samples[i - 1];
            sum_diff += diff < 0 ? -diff : diff;
            if (((sample < 0) && (samples[i - 1] >= 0)) || ((sample >= 0) && (samples[i - 1] < 0))) {
                zero_cross++;
            }
        }
    }

    int64_t avg_abs = sample_count ? (sum_abs / (int64_t)sample_count) : 0;
    int64_t avg_diff = (sample_count > 1U) ? (sum_diff / (int64_t)(sample_count - 1U)) : 0;
    int64_t mean = sample_count ? (sum / (int64_t)sample_count) : 0;
    int64_t centered_abs = 0;
    for (size_t i = 0; i < sample_count; ++i) {
        int64_t centered = (int64_t)samples[i] - mean;
        centered_abs += centered < 0 ? -centered : centered;
    }
    centered_abs = sample_count ? (centered_abs / (int64_t)sample_count) : 0;
    int32_t span = (max_sample >= min_sample) ? (max_sample - min_sample) : 0;
    if (peak_out) {
        *peak_out = peak;
    }
    if (avg_abs_out) {
        *avg_abs_out = avg_abs;
    }

    return (centered_abs * 24) + (avg_diff * 6) + ((int64_t)span * 2) + ((int64_t)zero_cross * 128)
           - ((mean < 0 ? -mean : mean) * 20) - ((int64_t)clipped * 4096);
}

static const char *call_status_to_str(esp_hf_call_status_t status)
{
    switch (status) {
    case ESP_HF_CALL_STATUS_NO_CALLS:
        return "NO_CALLS";
    case ESP_HF_CALL_STATUS_CALL_IN_PROGRESS:
        return "CALL_IN_PROGRESS";
    default:
        return "UNKNOWN";
    }
}

static const char *call_setup_to_str(esp_hf_call_setup_status_t status)
{
    switch (status) {
    case ESP_HF_CALL_SETUP_STATUS_IDLE:
        return "IDLE";
    case ESP_HF_CALL_SETUP_STATUS_INCOMING:
        return "INCOMING";
    case ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING:
        return "OUTGOING_DIALING";
    case ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING:
        return "OUTGOING_ALERTING";
    default:
        return "UNKNOWN";
    }
}

static const char *call_held_to_str(esp_hf_call_held_status_t status)
{
    switch (status) {
    case ESP_HF_CALL_HELD_STATUS_NONE:
        return "NONE";
    case ESP_HF_CALL_HELD_STATUS_HELD_AND_ACTIVE:
        return "HELD_AND_ACTIVE";
    case ESP_HF_CALL_HELD_STATUS_HELD:
        return "HELD";
    default:
        return "UNKNOWN";
    }
}

static void hf_client_audio_data_cb(esp_hf_sync_conn_hdl_t sync_conn_hdl, esp_hf_audio_buff_t *audio_buf, bool is_bad_frame)
{
    if (!s_audio_connected || audio_buf == NULL) {
        return;
    }

    const uint8_t *buf = audio_buf->data;
    uint32_t len = audio_buf->data_len;
    if (is_bad_frame || buf == NULL || len == 0) {
        esp_hf_client_audio_buff_free(audio_buf);
        return;
    }
    if (!s_cvsd_decoders_ready) {
        reset_cvsd_state();
    }

#if HFP_RX_FORCE_BYTESWAP_PCM
    int16_t pcm_samples[120];
    size_t pcm_count = len / 2U;
    if (pcm_count > (sizeof(pcm_samples) / sizeof(pcm_samples[0]))) {
        pcm_count = sizeof(pcm_samples) / sizeof(pcm_samples[0]);
    }

    for (size_t i = 0; i < pcm_count; ++i) {
        uint16_t hi = buf[(i * 2U) + 0U];
        uint16_t lo = buf[(i * 2U) + 1U];
        int16_t full_sample = (int16_t)((hi << 8) | lo);
        int16_t coarse_sample = (int16_t)(((int16_t)((int8_t)hi)) << 8);
        pcm_samples[i] = (int16_t)(((int32_t)coarse_sample * 3 + (int32_t)full_sample) / 4);
    }

    s_rx_packet_count++;
    if (s_rx_packet_count == 1U) {
        ESP_LOGI(TAG, "Using byte-swapped PCM-like path with 3:1 coarse/full blend");
    }
    if (s_pcm_snapshot_count < 3U) {
        log_cvsd_snapshot(buf, len, pcm_samples, pcm_count, 99);
        s_pcm_snapshot_count++;
    }

    int32_t peak = 0;
    int64_t avg_abs = 0;
    (void)score_pcm_candidate(pcm_samples, pcm_count, &peak, &avg_abs);

    if (avg_abs < 1500) {
        memset(pcm_samples, 0, pcm_count * sizeof(pcm_samples[0]));
    } else if (avg_abs < 5000) {
        for (size_t i = 0; i < pcm_count; ++i) {
            pcm_samples[i] = (int16_t)(pcm_samples[i] / 2);
        }
    }

    if ((s_rx_packet_count % 200U) == 0U) {
        ESP_LOGI(
            TAG,
            "HFP RX stats: packets=%" PRIu32 " mode=byteswap-blend peak=%" PRId32 " avg_abs=%" PRIi64 " samples=%u raw_bytes=%u",
            s_rx_packet_count,
            peak,
            avg_abs,
            (unsigned)pcm_count,
            (unsigned)len);
    }

    if (avg_abs >= 2500) {
        s_rx_voice_active_count++;
        if ((s_rx_voice_active_count % 50U) == 0U) {
            ESP_LOGI(
                TAG,
                "RX voice likely present: avg_abs=%" PRIi64 " peak=%" PRId32 " (active_count=%" PRIu32 ")",
                avg_abs,
                peak,
                s_rx_voice_active_count);
        }
    } else {
        s_rx_voice_silent_count++;
        if ((s_rx_voice_silent_count % 200U) == 0U) {
            ESP_LOGI(
                TAG,
                "RX mostly quiet/noise floor: avg_abs=%" PRIi64 " peak=%" PRId32 " (quiet_count=%" PRIu32 ")",
                avg_abs,
                peak,
                s_rx_voice_silent_count);
        }
    }

    audio_route_handle_hfp_rx_pcm(pcm_samples, pcm_count);
#else
    int16_t decoded_candidates[4][120];
    size_t decoded_counts[4] = {0};
    int best_mode = CVSD_FORCED_MODE;
    int64_t best_score = INT64_MIN;
    int32_t best_peak = 0;
    int64_t best_avg_abs = 0;

    for (size_t mode = 0; mode < (sizeof(s_cvsd_modes) / sizeof(s_cvsd_modes[0])); ++mode) {
        decoded_counts[mode] = cvsd_decode_packet_ex(
            &s_cvsd_decoders[mode],
            buf,
            len,
            decoded_candidates[mode],
            sizeof(decoded_candidates[mode]) / sizeof(decoded_candidates[mode][0]),
            s_cvsd_modes[mode].lsb_first,
            s_cvsd_modes[mode].invert_symbol);

        int32_t peak = 0;
        int64_t avg_abs = 0;
        int64_t score = score_pcm_candidate(decoded_candidates[mode], decoded_counts[mode], &peak, &avg_abs);
        if (score > best_score) {
            best_score = score;
            best_mode = (int)mode;
            best_peak = peak;
            best_avg_abs = avg_abs;
        }
    }

    s_rx_packet_count++;
    if (s_rx_packet_count == 1U) {
        ESP_LOGI(
            TAG,
            "Forcing CVSD mode=%d (lsb_first=%d invert=%d)",
            CVSD_FORCED_MODE,
            s_cvsd_modes[CVSD_FORCED_MODE].lsb_first,
            s_cvsd_modes[CVSD_FORCED_MODE].invert_symbol);
    }
    best_mode = CVSD_FORCED_MODE;
    (void)score_pcm_candidate(decoded_candidates[best_mode], decoded_counts[best_mode], &best_peak, &best_avg_abs);

    if (s_pcm_snapshot_count < 3U) {
        log_cvsd_snapshot(buf, len, decoded_candidates[best_mode], decoded_counts[best_mode], best_mode);
        s_pcm_snapshot_count++;
    }

    if ((s_rx_packet_count % 200U) == 0U) {
        ESP_LOGI(
            TAG,
            "HFP RX stats: packets=%" PRIu32 " mode=%d peak=%" PRId32 " avg_abs=%" PRIi64 " samples=%u raw_bytes=%u",
            s_rx_packet_count,
            best_mode,
            best_peak,
            best_avg_abs,
            (unsigned)decoded_counts[best_mode],
            (unsigned)len);
    }

    audio_route_handle_hfp_rx_pcm(decoded_candidates[best_mode], decoded_counts[best_mode]);
#endif
    esp_hf_audio_buff_t *tx_buf = esp_hf_client_audio_buff_alloc(len);
    if (tx_buf != NULL && tx_buf->data != NULL && tx_buf->buff_size >= len) {
        int16_t mic_samples[120] = {0};
        size_t mic_sample_count = len;
        if (mic_sample_count > (sizeof(mic_samples) / sizeof(mic_samples[0]))) {
            mic_sample_count = sizeof(mic_samples) / sizeof(mic_samples[0]);
        }
        size_t captured_samples = audio_route_take_mic_pcm(mic_samples, mic_sample_count, 0);

        int32_t tx_peak = 0;
        int64_t tx_avg_abs = 0;
        (void)score_pcm_candidate(mic_samples, captured_samples, &tx_peak, &tx_avg_abs);

        tx_buf->data_len = len;
        memset(tx_buf->data, 0x00, len);
#if HFP_TX_ENABLE_MIC_UPLINK
        int16_t upsampled_samples[120] = {0};
        size_t tx_pcm_samples = len;
        if (tx_pcm_samples > (sizeof(upsampled_samples) / sizeof(upsampled_samples[0]))) {
            tx_pcm_samples = sizeof(upsampled_samples) / sizeof(upsampled_samples[0]);
        }
        for (size_t i = 0; i < tx_pcm_samples; ++i) {
            int16_t sample = 0;
            if (captured_samples > 0) {
                sample = mic_samples[(i * captured_samples) / tx_pcm_samples];
            }
            int32_t scaled = (int32_t)sample * 8;
            if (scaled > INT16_MAX) {
                scaled = INT16_MAX;
            } else if (scaled < INT16_MIN) {
                scaled = INT16_MIN;
            }
            upsampled_samples[i] = (int16_t)scaled;
        }

        size_t encoded_bytes = 0;
        if (s_cvsd_encoder_ready) {
            encoded_bytes = cvsd_encode_packet(&s_cvsd_encoder, upsampled_samples, tx_pcm_samples, tx_buf->data, len);
        }
        if (encoded_bytes < len) {
            memset(tx_buf->data + encoded_bytes, 0x55, len - encoded_bytes);
        }
#endif

        s_tx_packet_count++;
        if (s_tx_packet_count == 1U && !HFP_TX_ENABLE_MIC_UPLINK) {
            ESP_LOGI(TAG, "Phase 3 diagnostic mode: mic capture is logged, HFP TX sends silence");
        }
        if ((s_tx_packet_count % 200U) == 0U) {
            ESP_LOGI(
                TAG,
                "HFP TX stats: packets=%" PRIu32 " peak=%" PRId32 " avg_abs=%" PRIi64 " captured_samples=%u payload_bytes=%u",
                s_tx_packet_count,
                tx_peak,
                tx_avg_abs,
                (unsigned)captured_samples,
                (unsigned)len);
        }

        if (esp_hf_client_audio_data_send(sync_conn_hdl, tx_buf) != ESP_OK) {
            esp_hf_client_audio_buff_free(tx_buf);
        }
    }

    esp_hf_client_audio_buff_free(audio_buf);
}

static void format_bda(const esp_bd_addr_t bda, char *buffer, size_t buffer_len)
{
    snprintf(
        buffer,
        buffer_len,
        "%02X:%02X:%02X:%02X:%02X:%02X",
        bda[0],
        bda[1],
        bda[2],
        bda[3],
        bda[4],
        bda[5]);
}

static const char *hf_conn_state_to_str(esp_hf_client_connection_state_t state)
{
    switch (state) {
    case ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED:
        return "DISCONNECTED";
    case ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING:
        return "CONNECTING";
    case ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED:
        return "CONNECTED";
    case ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED:
        return "SLC_CONNECTED";
    case ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTING:
        return "DISCONNECTING";
    default:
        return "UNKNOWN";
    }
}

static const char *hf_audio_state_to_str(esp_hf_client_audio_state_t state)
{
    switch (state) {
    case ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED:
        return "DISCONNECTED";
    case ESP_HF_CLIENT_AUDIO_STATE_CONNECTING:
        return "CONNECTING";
    case ESP_HF_CLIENT_AUDIO_STATE_CONNECTED:
        return "CONNECTED";
    case ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC:
        return "CONNECTED_MSBC";
    default:
        return "UNKNOWN";
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            char bda_str[18] = {0};
            format_bda(param->auth_cmpl.bda, bda_str, sizeof(bda_str));
            ESP_LOGI(TAG, "Pairing complete: %s (%s)", bda_str, param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Pairing failed, status=%d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT: {
        esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
        ESP_LOGW(TAG, "Legacy PIN requested, replying with 1234");
        ESP_ERROR_CHECK(esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code));
        break;
    }

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP confirmation requested, numeric value=%lu", (unsigned long)param->cfm_req.num_val);
        ESP_ERROR_CHECK(esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true));
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "Passkey notification: %lu", (unsigned long)param->key_notif.passkey);
        break;

    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "Passkey entry requested by peer");
        break;

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "Mode change: mode=%d interval=%d", param->mode_chg.mode, param->mode_chg.interval);
        break;

    default:
        ESP_LOGI(TAG, "Unhandled GAP event: %d", event);
        break;
    }
}

static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    switch (event) {
    case ESP_HF_CLIENT_PROF_STATE_EVT:
        ESP_LOGI(TAG, "HFP profile state: %d", param->prof_stat.state);
        break;

    case ESP_HF_CLIENT_CONNECTION_STATE_EVT: {
        char bda_str[18] = {0};
        format_bda(param->conn_stat.remote_bda, bda_str, sizeof(bda_str));
        ESP_LOGI(
            TAG,
            "HFP connection state: %s, peer=%s, peer_feat=0x%08" PRIx32 ", chld_feat=0x%08" PRIx32,
            hf_conn_state_to_str(param->conn_stat.state),
            bda_str,
            param->conn_stat.peer_feat,
            param->conn_stat.chld_feat);
        break;
    }

    case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
        char bda_str[18] = {0};
        format_bda(param->audio_stat.remote_bda, bda_str, sizeof(bda_str));
        ESP_LOGI(
            TAG,
            "HFP audio state: %s, peer=%s, sync_handle=%u, frame_size=%u",
            hf_audio_state_to_str(param->audio_stat.state),
            bda_str,
            (unsigned int)param->audio_stat.sync_conn_handle,
            (unsigned int)param->audio_stat.preferred_frame_size);

        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
            param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                ESP_LOGW(TAG, "mSBC is not supported in the current playback pipeline; expected CVSD-only operation");
            }
            reset_cvsd_state();
            s_audio_connected = true;
            s_sync_conn_hdl = param->audio_stat.sync_conn_handle;
            if (!s_data_cb_registered) {
                esp_err_t err = esp_hf_client_register_audio_data_callback(hf_client_audio_data_cb);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "esp_hf_client_register_audio_data_callback failed: %s", esp_err_to_name(err));
                } else {
                    s_data_cb_registered = true;
                }
            }
            esp_err_t err = audio_route_start_playback();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "audio_route_start_playback failed: %s", esp_err_to_name(err));
            }
            err = audio_route_start_capture();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "audio_route_start_capture failed: %s", esp_err_to_name(err));
            }
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
            s_audio_connected = false;
            s_sync_conn_hdl = 0;
            reset_cvsd_state();
            esp_err_t err = audio_route_stop_playback();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "audio_route_stop_playback failed: %s", esp_err_to_name(err));
            }
            err = audio_route_stop_capture();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "audio_route_stop_capture failed: %s", esp_err_to_name(err));
            }
        }
        break;
    }

    case ESP_HF_CLIENT_CIND_CALL_EVT:
        ESP_LOGI(TAG, "Call state: %s", call_status_to_str(param->call.status));
        break;

    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        ESP_LOGI(TAG, "Call setup state: %s", call_setup_to_str(param->call_setup.status));
        break;

    case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        ESP_LOGI(TAG, "Call held state: %s", call_held_to_str(param->call_held.status));
        break;

    case ESP_HF_CLIENT_CLIP_EVT:
        ESP_LOGI(TAG, "Incoming caller ID: %s", param->clip.number ? param->clip.number : "(none)");
        break;

    case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "Volume control: target=%d volume=%d", param->volume_control.type, param->volume_control.volume);
        break;

    case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        ESP_LOGI(TAG, "AT response: code=%d cme=%d", param->at_response.code, param->at_response.cme);
        break;

    case ESP_HF_CLIENT_BVRA_EVT:
        ESP_LOGI(TAG, "Voice recognition state: %d", param->bvra.value);
        break;

    case ESP_HF_CLIENT_RING_IND_EVT:
        ESP_LOGI(TAG, "Ring indication received");
        break;

    default:
        ESP_LOGI(TAG, "Unhandled HFP event: %d", event);
        break;
    }
}

esp_err_t bt_hfp_init(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;

    ESP_LOGI(TAG, "Initializing Bluetooth controller");

    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BLE memory release returned %s", esp_err_to_name(err));
    }

    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&bt_cfg), TAG, "esp_bt_controller_init failed");
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT), TAG, "esp_bt_controller_enable failed");
    ESP_RETURN_ON_ERROR(esp_bluedroid_init(), TAG, "esp_bluedroid_init failed");
    ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "esp_bluedroid_enable failed");

    ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(gap_cb), TAG, "esp_bt_gap_register_callback failed");
    ESP_RETURN_ON_ERROR(esp_bt_gap_set_device_name(DEVICE_NAME), TAG, "esp_bt_gap_set_device_name failed");
    ESP_RETURN_ON_ERROR(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE), TAG, "esp_bt_gap_set_scan_mode failed");
    ESP_RETURN_ON_ERROR(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap)), TAG, "esp_bt_gap_set_security_param failed");
    ESP_RETURN_ON_ERROR(esp_bt_gap_set_pin(pin_type, 0, NULL), TAG, "esp_bt_gap_set_pin failed");

    ESP_RETURN_ON_ERROR(esp_hf_client_register_callback(hf_client_cb), TAG, "esp_hf_client_register_callback failed");
    ESP_RETURN_ON_ERROR(esp_hf_client_init(), TAG, "esp_hf_client_init failed");
    ESP_LOGI(TAG, "Bluetooth Classic ready");
    ESP_LOGI(TAG, "Device is discoverable/connectable as '%s'", DEVICE_NAME);
    ESP_LOGI(TAG, "Phase 2 flow: pair from phone -> connect HFP -> place call -> observe AUDIO_STATE and playback logs");
    ESP_LOGI(TAG, "Current implementation handles HFP RX audio playback only; microphone uplink remains Phase 3");

    return ESP_OK;
}
