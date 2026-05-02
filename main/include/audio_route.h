#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t audio_route_init(void);
esp_err_t audio_route_start_playback(void);
esp_err_t audio_route_stop_playback(void);
esp_err_t audio_route_start_capture(void);
esp_err_t audio_route_stop_capture(void);
size_t audio_route_take_mic_pcm(int16_t *samples, size_t max_samples, uint32_t timeout_ms);
void audio_route_handle_hfp_rx_pcm(const int16_t *samples, size_t sample_count);
