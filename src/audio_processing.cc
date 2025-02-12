#include "rel_common.h"
#include "audio_processing.h"
#include "processing_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_dsp.h"
#include "driver/adc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_heap_caps.h"
#include "driver/i2s.h"

#include <math.h>

#define SAMPLE_RATE 22500
#define RECORD_TIME 6000
#define BUFFER_SIZE 4096
#define FRAME_LENGTH 512
#define FRAME_STEP 256
#define NUM_MEL_FILTERS 40
#define FFT_SIZE 512
#define SAMPLE_INTERVAL (1000000 / SAMPLE_RATE)
#define MAX_AUDIO_SIZE (SAMPLE_RATE * RECORD_TIME / 1000)
#define ADC_CHANNEL ADC_CHANNEL_1

static const char* TAG = "AUDIO_PROCESSING";

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc_cali_handle = NULL;

static float* fbank;
float* fft_buffer;

void apply_mel_filterbank(float* spectrum, float* mel_energies, float* fbank, int n_filters, int n_fft);

esp_err_t feature_extractor(FILE* audio_file, float* mfcc, int n_mfcc) {
    fseek(audio_file, 44, SEEK_SET);

    int16_t* audio_data = (int16_t*)heap_caps_malloc(MAX_AUDIO_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!audio_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for audio data");
        return ESP_ERR_NO_MEM;
    }

    int frame_count = 0;

    size_t audio_size = fread(audio_data, sizeof(int16_t), MAX_AUDIO_SIZE, audio_file);

    float* frame_real = (float*)heap_caps_malloc(FRAME_LENGTH * sizeof(float), MALLOC_CAP_SPIRAM);
    float* frame_imag = (float*)heap_caps_malloc(FRAME_LENGTH * sizeof(float), MALLOC_CAP_SPIRAM);
    float* mel_energies = (float*)heap_caps_malloc(NUM_MEL_FILTERS * sizeof(float), MALLOC_CAP_SPIRAM);

    for (int i = 0; i < audio_size - FRAME_LENGTH; i += FRAME_STEP) {
        for (int j = 0; j < FRAME_LENGTH; j++) {
            frame_real[j] = (float)audio_data[i + j] / 32768.0f;
            frame_imag[j] = 0.0f;
        }

        // 프리엠퍼시스
        dsps_preemphasis(frame_real, frame_real, FRAME_LENGTH, 0.97f);

        // 윈도우 적용
        dsps_wind_hann_f32(frame_real, FRAME_LENGTH);

        //FFT 수행
        fft(frame_real, frame_imag, FRAME_LENGTH);

        // 멜 필터뱅크 적용
        for (int j = 0; j < FRAME_LENGTH / 2 + 1; j++) {
            float magnitude = sqrtf(frame_real[j] * frame_real[j] + frame_imag[j] * frame_imag[j]);
            frame_real[j] = magnitude;
        }
        apply_mel_filterbank(frame_real, mel_energies, fbank, NUM_MEL_FILTERS, FFT_SIZE / 2 + 1);

        // 로그 변환
        dsps_log(mel_energies, NUM_MEL_FILTERS);

        // DCT 수행
        dsps_dct_f32(mel_energies, n_mfcc);

        for (int j = 0; j < n_mfcc; j++) {
            mfcc[j] += mel_energies[j];
        }

        frame_count++;
    }

    if (frame_count > 0) {
        for (int i = 0; i < n_mfcc; i++) {
            mfcc[i] /= frame_count;
        }
    }

    heap_caps_free(audio_data);
    heap_caps_free(frame_real);
    heap_caps_free(frame_imag);
    heap_caps_free(mel_energies);
    return ESP_OK;
}


esp_err_t init_audio_processing() {
    esp_err_t ret;

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    
    ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC calibration scheme. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    fbank = (float*)heap_caps_malloc(NUM_MEL_FILTERS * (FFT_SIZE / 2 + 1) * sizeof(float), MALLOC_CAP_SPIRAM);
    create_mel_filterbank(fbank, NUM_MEL_FILTERS, FFT_SIZE, SAMPLE_RATE);

    return ESP_OK;
}

void cleanup_audio_processing() {
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
    }
    if (adc_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
    }
    heap_caps_free(fbank);
}

void apply_mel_filterbank(float* spectrum, float* mel_energies, float* fbank, int n_filters, int n_fft) {
    for (int i = 0; i < n_filters; i++) {
        mel_energies[i] = 0.0f;
        for (int j = 0; j < n_fft / 2 + 1; j++) {
            mel_energies[i] += spectrum[j] * fbank[i * n_fft + j];
        }
    }
}

void scaler(float* features, int size, const char* scaler_path) {
    FILE* scaler_file = fopen(scaler_path, "rb");
    if (!scaler_file) {
        ESP_LOGE(TAG, "Failed to open scaler file");
        return;
    }

    float mean, std;
    fread(&mean, sizeof(float), 1, scaler_file);
    fread(&std, sizeof(float), 1, scaler_file);
    fclose(scaler_file);

    for (int i = 0; i < size; i++) {
        features[i] = (features[i] - mean) / std;
    }
}

void differential_mfcc(float* mfcc_features, float* delta_mfccs, float* delta2_mfccs, int size) {
    // 1차 미분
    dsps_diff(mfcc_features, delta_mfccs, size, 1);

    // 2차 미분
    dsps_diff(delta_mfccs, delta2_mfccs, size, 1);
}

void recordAudio() {
    FILE* f = fopen("/sdcard/audio.wav", "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    for (int i = 0; i < 44; i++) {
        fputc(0, f);
    }

    ESP_LOGI(TAG, "Recording started...");
    int64_t startTime = esp_timer_get_time();
    int64_t nextSampleTime = startTime;
    uint32_t totalSamples = 0;

    int16_t* audioBuffer = (int16_t*)heap_caps_malloc(BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!audioBuffer) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        fclose(f);
        return;
    }

    int bufferIndex = 0;

    while (esp_timer_get_time() - startTime < RECORD_TIME * 1000) {
        int64_t currentTime = esp_timer_get_time();
        if (currentTime >= nextSampleTime) {
            int adc_raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw));
            int voltage_mv;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv));
            int16_t sample = (int16_t)map(voltage_mv, 0, 3300, -32768, 32767);

            audioBuffer[bufferIndex++] = sample;
            totalSamples++;

            if (bufferIndex >= BUFFER_SIZE) {
                fwrite(audioBuffer, sizeof(int16_t), BUFFER_SIZE, f);
                bufferIndex = 0;
            }

            nextSampleTime += SAMPLE_INTERVAL;
        }
    }

    if (bufferIndex > 0) {
        fwrite(audioBuffer, sizeof(int16_t), bufferIndex, f);
    }

    heap_caps_free(audioBuffer);

    uint32_t dataSize = totalSamples * 2;
    writeWaveHeader(f, dataSize);

    fclose(f);
    ESP_LOGI(TAG, "Recording completed and saved");
}

void writeWaveHeader(FILE* file, uint32_t dataSize) {
    fseek(file, 0, SEEK_SET);
    
    fwrite("RIFF", 1, 4, file);
    uint32_t fileSize = dataSize + 36;
    fwrite(&fileSize, 1, 4, file);
    fwrite("WAVE", 1, 4, file);
    fwrite("fmt ", 1, 4, file);
    uint32_t fmtSize = 16;
    fwrite(&fmtSize, 1, 4, file);
    uint16_t audioFormat = 1;
    fwrite(&audioFormat, 1, 2, file);
    uint16_t numChannels = 1;
    fwrite(&numChannels, 1, 2, file);
    uint32_t sampleRate = SAMPLE_RATE;
    fwrite(&sampleRate, 1, 4, file);
    uint32_t byteRate = SAMPLE_RATE * 2;
    fwrite(&byteRate, 1, 4, file);
    uint16_t blockAlign = 2;
    fwrite(&blockAlign, 1, 2, file);
    uint16_t bitsPerSample = 16;
    fwrite(&bitsPerSample, 1, 2, file);
    fwrite("data", 1, 4, file);
    fwrite(&dataSize, 1, 4, file);
}