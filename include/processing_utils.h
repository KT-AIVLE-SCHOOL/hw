#ifndef PROCESSING_UTILS_H
#define PROCESSING_UTILS_H

#include <cstdint>
#include <ccomplex>

void create_mel_filterbank(float* fbank, int n_filters, int n_fft, float sample_rate);

void dsps_preemphasis(float* input, float* output, int length, float coeff);

void dsps_diff(float* input, float* output, int size, int step);

void dsps_log(float* input, int filter_size);

void fft(float *real, float *imag, int n);

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

float hz_to_mel(float hz);

float mel_to_hz(float mel);

#endif