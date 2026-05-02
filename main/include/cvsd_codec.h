#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t predictor;
    int32_t step_size;
    uint8_t last_symbol;
    uint8_t run_length;
    int32_t decim_accum;
    uint8_t decim_phase;
} cvsd_decoder_t;

typedef struct {
    int32_t predictor;
    int32_t step_size;
    uint8_t last_symbol;
    uint8_t run_length;
} cvsd_encoder_t;

typedef struct {
    bool lsb_first;
    bool invert_symbol;
} cvsd_mode_t;

void cvsd_encoder_init(cvsd_encoder_t *encoder);
size_t cvsd_encode_packet_ex(
    cvsd_encoder_t *encoder,
    const int16_t *input,
    size_t input_samples,
    uint8_t *output,
    size_t output_capacity,
    bool lsb_first,
    bool invert_symbol);
size_t cvsd_encode_packet(cvsd_encoder_t *encoder, const int16_t *input, size_t input_samples, uint8_t *output, size_t output_capacity);

void cvsd_decoder_init(cvsd_decoder_t *decoder);
size_t cvsd_decode_packet_ex(
    cvsd_decoder_t *decoder,
    const uint8_t *input,
    size_t input_len,
    int16_t *output,
    size_t output_capacity,
    bool lsb_first,
    bool invert_symbol);
size_t cvsd_decode_packet(cvsd_decoder_t *decoder, const uint8_t *input, size_t input_len, int16_t *output, size_t output_capacity);
