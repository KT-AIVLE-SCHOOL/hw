#include "rel_common.h"
#include "model_inference.h"
#include "audio_processing.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char* TAG = "MODEL_INFERENCE";

tflite::MicroMutableOpResolver<3> resolver1;
tflite::MicroMutableOpResolver<4> resolver2;
static tflite::MicroInterpreter* interpreter;

esp_err_t init_model_inference() {
    resolver1.AddFullyConnected();
    resolver1.AddLeakyRelu();
    resolver1.AddLogistic();

    resolver2.AddFullyConnected();
    resolver2.AddLeakyRelu();
    resolver2.AddSoftmax();

    return ESP_OK;
}

void cleanup_model_inference() {
    // 필요한 경우 모델 관련 리소스 정리
}

esp_err_t model_predict(FILE* audio_file, const char* model_path, int feature_num) {
    const int kTensorArenaSize = 250 * 1024;
    float* features = (float*)heap_caps_malloc(feature_num * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!features) {
        ESP_LOGE(TAG, "Failed to allocate features");
        return ESP_FAIL;
    }

    uint8_t* tensor_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM);
    if (!tensor_arena) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        heap_caps_free(features);
        return ESP_FAIL;
    }

    FILE* model_file = fopen(model_path, "rb");
    if (!model_file) {
        ESP_LOGE(TAG, "Failed to open model file");
        heap_caps_free(features);
        heap_caps_free(tensor_arena);
        return ESP_FAIL;
    }

    fseek(model_file, 0, SEEK_END);
    long model_size = ftell(model_file);
    fseek(model_file, 0, SEEK_SET);

    uint8_t* model_data = (uint8_t*)heap_caps_malloc(model_size, MALLOC_CAP_SPIRAM);
    if (!model_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for model");
        fclose(model_file);
        heap_caps_free(features);
        heap_caps_free(tensor_arena);
        return ESP_FAIL;
    }

    fread(model_data, 1, model_size, model_file);
    fclose(model_file);

    const tflite::Model* model = tflite::GetModel(model_data);
    if (feature_num == 120)
        interpreter = new tflite::MicroInterpreter(model, resolver1, tensor_arena, kTensorArenaSize, nullptr, nullptr);
    else
        interpreter = new tflite::MicroInterpreter(model, resolver2, tensor_arena, kTensorArenaSize, nullptr, nullptr);
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "Failed to allocate tensors");
        heap_caps_free(features);
        heap_caps_free(tensor_arena);
        heap_caps_free(model_data);
        delete interpreter;
        return ESP_FAIL;
    }

    esp_err_t ret;
    if (feature_num == 120) {
        ret = process1(audio_file, features);
    } else {
        ret = process2(audio_file, features);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio processing failed");
        heap_caps_free(features);
        heap_caps_free(tensor_arena);
        heap_caps_free(model_data);
        delete interpreter;
        return ESP_FAIL;
    }

    memcpy(interpreter->input(0)->data.f, features, feature_num * sizeof(float));

    if (interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Inference failed");
        heap_caps_free(features);
        heap_caps_free(tensor_arena);
        heap_caps_free(model_data);
        delete interpreter;
        return ESP_FAIL;
    }

    float* output = interpreter->output(0)->data.f;
    int output_size = interpreter->output(0)->dims->data[1];
    int result = std::distance(output, std::max_element(output, output + output_size));

    heap_caps_free(features);
    heap_caps_free(tensor_arena);
    heap_caps_free(model_data);
    delete interpreter;

    return result;
}

esp_err_t process1(FILE* audio_file, float* features) {
    float* mfcc = (float*)heap_caps_malloc(40 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* delta_mfccs = (float*)heap_caps_calloc(40, sizeof(float), MALLOC_CAP_SPIRAM);
    float* delta2_mfccs = (float*)heap_caps_calloc(40, sizeof(float), MALLOC_CAP_SPIRAM);

    if (!mfcc || !delta_mfccs || !delta2_mfccs) {
        ESP_LOGE(TAG, "Failed to allocate memory for MFCC");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = feature_extractor(audio_file, mfcc, 40);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Feature extraction failed");
        goto cleanup;
    }

    scaler(mfcc, 40, "/sdcard/first_model_scaler.pkl");
    differential_mfcc(mfcc, delta_mfccs, delta2_mfccs, 40);

    memcpy(features, mfcc, 40 * sizeof(float));
    memcpy(features + 40, delta_mfccs, 40 * sizeof(float));
    memcpy(features + 80, delta2_mfccs, 40 * sizeof(float));

cleanup:
    heap_caps_free(mfcc);
    heap_caps_free(delta_mfccs);
    heap_caps_free(delta2_mfccs);
    return ret;
}

esp_err_t process2(FILE* audio_file, float* features) {
    float* mfcc = (float*)heap_caps_malloc(80 * sizeof(float), MALLOC_CAP_SPIRAM);
    float* delta_mfccs = (float*)heap_caps_calloc(80, sizeof(float), MALLOC_CAP_SPIRAM);
    float* delta2_mfccs = (float*)heap_caps_calloc(80, sizeof(float), MALLOC_CAP_SPIRAM);

    if (!mfcc || !delta_mfccs || !delta2_mfccs) {
        ESP_LOGE(TAG, "Failed to allocate memory for MFCC");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = feature_extractor(audio_file, mfcc, 80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Feature extraction failed");
        goto cleanup;
    }

    differential_mfcc(mfcc, delta_mfccs, delta2_mfccs, 80);

    memcpy(features, mfcc, 80 * sizeof(float));
    memcpy(features + 80, delta_mfccs, 80 * sizeof(float));
    memcpy(features + 160, delta2_mfccs, 80 * sizeof(float));

    scaler(features, 240, "/sdcard/second_model_scaler.pkl");

cleanup:
    heap_caps_free(mfcc);
    heap_caps_free(delta_mfccs);
    heap_caps_free(delta2_mfccs);
    return ret;
}

const char* pipeline() {
    FILE* audio_file = fopen("/sdcard/audio.wav", "rb");
    if (!audio_file) {
        ESP_LOGE(TAG, "Failed to open audio file");
        return "-1";
    }

    int pred = model_predict(audio_file, "/sdcard/converted_first_model.tflite", 120);
    const char* answer;

    if (pred == -1) {
        ESP_LOGE(TAG, "Error in prediction");
        answer = "6";
    } else if (!pred) {
        ESP_LOGI(TAG, "model : no pain");
        fseek(audio_file, 0, SEEK_SET);
        pred = model_predict(audio_file, "/sdcard/converted_second_model.tflite", 240);
        switch (pred) {
            case 0:
                ESP_LOGI(TAG, "model : Awake");
                answer = "1";
                break;
            case 1:
                ESP_LOGI(TAG, "model : Diaper");
                answer = "2";
                break;
            case 2:
                ESP_LOGI(TAG, "model : hug");
                answer = "3";
                break;
            case 3:
                ESP_LOGI(TAG, "model : Hungry");
                answer = "4";
                break;
            case 4:
                ESP_LOGI(TAG, "model : Sleepy");
                answer = "5";
                break;
            default:
                ESP_LOGI(TAG, "model : Wrong prediction");
                answer = "6";
        }
    } else {
        ESP_LOGI(TAG, "model : pain");
        answer = "0";
    }

    fclose(audio_file);
    return answer;
}