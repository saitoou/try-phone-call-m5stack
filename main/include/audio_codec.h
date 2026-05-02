#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t audio_codec_init(void);
esp_err_t audio_codec_start_playback(void);
esp_err_t audio_codec_stop_playback(void);
esp_err_t audio_codec_write_playback(const void *data, size_t len, size_t *bytes_written, uint32_t timeout_ms);
esp_err_t audio_codec_start_capture(void);
esp_err_t audio_codec_stop_capture(void);
esp_err_t audio_codec_read_capture(void *data, size_t len, size_t *bytes_read, uint32_t timeout_ms);
esp_err_t audio_codec_play_test_tone(uint32_t duration_ms);
