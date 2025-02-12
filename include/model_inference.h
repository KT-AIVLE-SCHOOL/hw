#ifndef MODEL_INFERENCE_H
#define MODEL_INFERENCE_H

#include "esp_err.h"

esp_err_t init_model_inference();
void cleanup_model_inference();
esp_err_t model_predict(FILE* audio_file, const char* model_path, int feature_num);
esp_err_t process1(FILE* audio_file, float* features);
esp_err_t process2(FILE* audio_file, float* features);
const char* pipeline();

#endif