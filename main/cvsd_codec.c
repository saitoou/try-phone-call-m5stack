#include "cvsd_codec.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CVSD_MIN_STEP_SIZE       10
#define CVSD_MAX_STEP_SIZE       1280
#define CVSD_PREDICTOR_LIMIT     32767
#define CVSD_HW_H                5
#define CVSD_HW_BETA             10
#define CVSD_HW_J                4
#define CVSD_HW_K                4

static inline int32_t clamp32(int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

void cvsd_encoder_init(cvsd_encoder_t *encoder)
{
    if (encoder == NULL) {
        return;
    }

    encoder->predictor = 0;
    encoder->step_size = CVSD_MIN_STEP_SIZE;
    encoder->last_symbol = 0;
    encoder->run_length = 0;
}

static inline void cvsd_update_step(int32_t *step_size, uint8_t *run_length, uint8_t *last_symbol, uint8_t symbol)
{
    if (symbol != *last_symbol) {
        *run_length = 1;
    } else if (*run_length < 255U) {
        (*run_length)++;
    }
    *last_symbol = symbol;

    if (*run_length >= CVSD_HW_J) {
        *step_size += CVSD_HW_BETA * (int32_t)(*run_length - CVSD_HW_J + 1U);
    } else {
        *step_size -= *step_size >> CVSD_HW_K;
    }
    *step_size = clamp32(*step_size, CVSD_MIN_STEP_SIZE, CVSD_MAX_STEP_SIZE);
}

size_t cvsd_encode_packet_ex(
    cvsd_encoder_t *encoder,
    const int16_t *input,
    size_t input_samples,
    uint8_t *output,
    size_t output_capacity,
    bool lsb_first,
    bool invert_symbol)
{
    if (encoder == NULL || input == NULL || output == NULL) {
        return 0;
    }

    size_t produced = 0;
    for (size_t i = 0; i < input_samples && produced < output_capacity; ++i) {
        uint8_t packed = 0;
        int32_t target = input[i];

        for (int bit_index = 0; bit_index < 8; ++bit_index) {
            uint8_t symbol = (target >= encoder->predictor) ? 1U : 0U;
            if (invert_symbol) {
                symbol ^= 0x01U;
            }

            int bit = lsb_first ? bit_index : (7 - bit_index);
            packed |= (uint8_t)(symbol << bit);

            uint8_t effective_symbol = invert_symbol ? (symbol ^ 0x01U) : symbol;
            cvsd_update_step(&encoder->step_size, &encoder->run_length, &encoder->last_symbol, effective_symbol);

            const int32_t delta = effective_symbol ? encoder->step_size : -encoder->step_size;
            encoder->predictor += (delta - encoder->predictor) / CVSD_HW_H;
            encoder->predictor = clamp32(encoder->predictor, -CVSD_PREDICTOR_LIMIT, CVSD_PREDICTOR_LIMIT);
        }

        output[produced++] = packed;
    }

    return produced;
}

size_t cvsd_encode_packet(cvsd_encoder_t *encoder, const int16_t *input, size_t input_samples, uint8_t *output, size_t output_capacity)
{
    return cvsd_encode_packet_ex(encoder, input, input_samples, output, output_capacity, true, false);
}

void cvsd_decoder_init(cvsd_decoder_t *decoder)
{
    if (decoder == NULL) {
        return;
    }

    decoder->predictor = 0;
    decoder->step_size = CVSD_MIN_STEP_SIZE;
    decoder->last_symbol = 0;
    decoder->run_length = 0;
    decoder->decim_accum = 0;
    decoder->decim_phase = 0;
}

size_t cvsd_decode_packet_ex(
    cvsd_decoder_t *decoder,
    const uint8_t *input,
    size_t input_len,
    int16_t *output,
    size_t output_capacity,
    bool lsb_first,
    bool invert_symbol)
{
    if (decoder == NULL || input == NULL || output == NULL) {
        return 0;
    }

    size_t produced = 0;
    for (size_t i = 0; i < input_len && produced < output_capacity; ++i) {
        uint8_t byte = input[i];
        for (int bit_index = 0; bit_index < 8 && produced < output_capacity; ++bit_index) {
            int bit = lsb_first ? bit_index : (7 - bit_index);
            uint8_t symbol = (byte >> bit) & 0x01;
            if (invert_symbol) {
                symbol ^= 0x01;
            }

            if ((i == 0U && bit_index == 0) || symbol != decoder->last_symbol) {
                decoder->run_length = 1;
            } else {
                if (decoder->run_length < 255) {
                    decoder->run_length++;
                }
            }
            decoder->last_symbol = symbol;

            if (decoder->run_length >= CVSD_HW_J) {
                decoder->step_size += CVSD_HW_BETA * (int32_t)(decoder->run_length - CVSD_HW_J + 1U);
            } else {
                decoder->step_size -= decoder->step_size >> CVSD_HW_K;
            }
            decoder->step_size = clamp32(decoder->step_size, CVSD_MIN_STEP_SIZE, CVSD_MAX_STEP_SIZE);

            const int32_t delta = symbol ? decoder->step_size : -decoder->step_size;
            decoder->predictor += (delta - decoder->predictor) / CVSD_HW_H;
            decoder->predictor = clamp32(decoder->predictor, -CVSD_PREDICTOR_LIMIT, CVSD_PREDICTOR_LIMIT);

            decoder->decim_accum += decoder->predictor;
            decoder->decim_phase++;
            /* Empirically, the ESP-IDF HFP external-codec callback delivers
             * 120-byte CVSD payloads at roughly 7.5-8 ms cadence on this setup.
             * To match the actual 8 kHz playback rate, one PCM sample must be
             * generated from 16 incoming symbols here. */
            if (decoder->decim_phase == 16) {
                output[produced++] = (int16_t)(decoder->decim_accum / 16);
                decoder->decim_accum = 0;
                decoder->decim_phase = 0;
            }
        }
    }

    return produced;
}

size_t cvsd_decode_packet(cvsd_decoder_t *decoder, const uint8_t *input, size_t input_len, int16_t *output, size_t output_capacity)
{
    return cvsd_decode_packet_ex(decoder, input, input_len, output, output_capacity, true, false);
}
