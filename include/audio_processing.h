#ifndef AUDIO_PROCESSING_HPP
#define AUDIO_PROCESSING_HPP

#include "esp_err.h"

esp_err_t feature_extractor(FILE* audio_file, float* mfcc, int n_mfcc);
void scaler(float* features, int size, const char* scaler_path);
void differential_mfcc(float* mfcc_features, float* delta_mfccs, float* delta2_mfccs, int size);
void recordAudio();
esp_err_t init_audio_processing();
void cleanup_audio_processing();
void writeWaveHeader(FILE* file, uint32_t dataSize);

#endif