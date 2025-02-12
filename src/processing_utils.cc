#include "processing_utils.h"
#include "esp_dsp.h"
#include "math.h"

void create_mel_filterbank(float* fbank, int n_filters, int n_fft, float sample_rate) {
    float fmin_mel = hz_to_mel(0);
    float fmax_mel = hz_to_mel(sample_rate / 2);
    float mel_step = (fmax_mel - fmin_mel) / (n_filters + 1);

    for (int i = 0; i < n_filters; i++) {
        float left_mel = fmin_mel + i * mel_step;
        float center_mel = left_mel + mel_step;
        float right_mel = center_mel + mel_step;

        int left_bin = (int)((n_fft + 1) * mel_to_hz(left_mel) / sample_rate);
        int center_bin = (int)((n_fft + 1) * mel_to_hz(center_mel) / sample_rate);
        int right_bin = (int)((n_fft + 1) * mel_to_hz(right_mel) / sample_rate);

        for (int j = left_bin; j < right_bin; j++) {
            if (j < center_bin) {
                fbank[i * n_fft + j] = (j - left_bin) / (float)(center_bin - left_bin);
            } else {
                fbank[i * n_fft + j] = (right_bin - j) / (float)(right_bin - center_bin);
            }
        }
    }
}

void dsps_preemphasis(float* input, float* output, int length, float coeff) {
    output[0] = input[0];
    for (int i = 1; i < length; i++) {
        output[i] = input[i] - coeff * input[i-1];
    }
}

void fft(float *real, float *imag, int n) {
    // 비트 리버스 순서로 데이터 재배열
    for (int i = 0, j = 0; i < n; i++) {
        if (j > i) {
            float temp_real = real[i];
            float temp_imag = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = temp_real;
            imag[j] = temp_imag;
        }
        int m = n >> 1;
        while (m >= 1 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // FFT 계산
    for (int s = 1; s < n; s <<= 1) {
        int m = s << 1;
        float wr = 1.0, wi = 0.0;
        float theta = -2 * M_PI / m;
        float wpr = cosf(theta), wpi = sinf(theta);
        for (int i = 0; i < n; i += m) {
            for (int j = i, k = i + s; j < i + s; j++, k++) {
                float tr = wr * real[k] - wi * imag[k];
                float ti = wr * imag[k] + wi * real[k];
                real[k] = real[j] - tr;
                imag[k] = imag[j] - ti;
                real[j] += tr;
                imag[j] += ti;
            }
            float wtemp = wr;
            wr = wr * wpr - wi * wpi;
            wi = wi * wpr + wtemp * wpi;
        }
    }
}

void dsps_diff(float* input, float* output, int size, int step) {
    for (int i = step; i < size; i++) {
        output[i] = input[i] - input[i-step];
    }
}

void dsps_log(float* input, int filter_size) {
    for (int i = 0; i < filter_size; i++) {
        input[i] = logf(input[i] + 1e-6);
    }
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float hz_to_mel(float hz) {
    return 2595.0 * log10f(1.0 + hz / 700.0);
}

float mel_to_hz(float mel) {
    return 700.0 * (powf(10, mel / 2595.0) - 1.0);
}