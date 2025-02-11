#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SD.h>
#include <SPI.h>
#include <FS.h>
#include <driver/adc.h>
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"

#define SAMPLE_RATE 16000
#define MIC_PIN ADC1_CHANNEL_8
#define RECORD_TIME 5000
#define CS_PIN 13
#define SCK_PIN 16
#define MOSI_PIN 19
#define MISO_PIN 41
#define BUFFER_SIZE 64
#define SAMPLE_INTERVAL 1000000 / SAMPLE_RATE
#define MAX_AUDIO_SIZE (SAMPLE_RATE * RECORD_TIME / 1000)

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

tflite::MicroErrorReporter error_reporter;
tflite::AllOpsResolver resolver;
tflite::MicroInterpreter* interpreter;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
  
  void onMtuChanged(BLEServer* pServer, uint16_t mtu) {
    Serial.printf("MTU 변경: %u\n", mtu);
  }
};

void setup() {
  Serial.begin(115200);

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(MIC_PIN, ADC_ATTEN_DB_11);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  if (!SD.begin(CS_PIN)) {
    Serial.println(F("SD 카드 초기화 실패"));
    return;
  }
  Serial.println(F("SD카드 초기화 성공"));

  if (psramInit()) {
    Serial.println(F("PSRAM initialized"));
  } else {
    Serial.println(F("PSRAM initialization failed"));
  }

  // BLE 초기화
  BLEDevice::init("bigAivleAudio");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println(F("BLE 서버 시작, 연결 대기 중..."));

  delay(1000);
}

void loop() {
  if (deviceConnected) {
    std::string stdValue = pCharacteristic->getValue();
    String value = String(stdValue.c_str());
    if (value == "r") {
      delay(500);
      pCharacteristic->setValue((uint8_t*)"w", strlen("w"));
      pCharacteristic->notify();
      recordAudio();
      pipeline();
    }
    delay(100);
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println(F("BLE 연결 해제, 광고 시작..."));
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println(F("BLE 연결 설정 됨"));
  }
}

inline void writeWavHeader(File& file, uint32_t dataSize) {
  file.seek(0);
  file.write((const uint8_t *)"RIFF", 4);
  uint32_t fileSize = dataSize + 36;
  file.write((const uint8_t *)&fileSize, 4);
  file.write((const uint8_t *)"WAVE", 4);
  file.write((const uint8_t *)"fmt ", 4);
  uint32_t fmtSize = 16;
  file.write((const uint8_t *)&fmtSize, 4);
  uint16_t audioFormat = 1;
  file.write((const uint8_t *)&audioFormat, 2);
  uint16_t numChannels = 1;
  file.write((const uint8_t *)&numChannels, 2);
  uint32_t sampleRate = SAMPLE_RATE;
  file.write((const uint8_t *)&sampleRate, 4);
  uint32_t byteRate = SAMPLE_RATE * 2;
  file.write((const uint8_t *)&byteRate, 4);
  uint16_t blockAlign = 2;
  file.write((const uint8_t *)&blockAlign, 2);
  uint16_t bitsPerSample = 16;
  file.write((const uint8_t *)&bitsPerSample, 2);
  file.write((const uint8_t *)"data", 4);
  file.write((const uint8_t *)&dataSize, 4);
}

void recordAudio() {
  File audioFile = SD.open("/audio.wav", FILE_WRITE);
  if (!audioFile) {
    Serial.println(F("파일 열기 실패"));
    return;
  }

  // WAV 헤더를 위한 공간 확보
  for (int i = 0; i < 44; i++) {
    audioFile.write((uint8_t)0);
  }

  Serial.println(F("녹음 시작"));
  unsigned long startTime = millis();
  unsigned long nextSampleTime = micros();
  uint32_t totalSamples = 0;

  int16_t* audioBuffer = (int16_t*)ps_malloc(BUFFER_SIZE * sizeof(int16_t));

  int bufferIndex = 0;

  while (millis() - startTime < RECORD_TIME) {
    if (micros() >= nextSampleTime) {
      int micValue = adc1_get_raw(MIC_PIN);
      int16_t sample = map(micValue, 0, 4095, -32768, 32767);

      audioBuffer[bufferIndex++] = micValue;
      totalSamples++;

      if (bufferIndex >= BUFFER_SIZE) {
        audioFile.write((const uint8_t*)audioBuffer, BUFFER_SIZE * sizeof(int16_t));
        bufferIndex = 0;
      }

      nextSampleTime += SAMPLE_INTERVAL;
    }
  }

  if (bufferIndex > 0) {
    audioFile.write((const uint8_t*)audioBuffer, bufferIndex * sizeof(int16_t));
  }

  free(audioBuffer);

  uint32_t dataSize = totalSamples * 2;
  writeWavHeader(audioFile, dataSize);

  audioFile.close();
  Serial.println(F("녹음 완료 및 저장"));
}

void feature_extractor(File& audioFile, float* mfcc, int n_mfcc) {
  audioFile.seek(44);

  const int maxAudioSize = SAMPLE_RATE * RECORD_TIME / 1000;
  static int16_t audioData[MAX_AUDIO_SIZE];
  if (!audioData) {
    Serial.println(F("오디오 데이터 메모리 할당 실패"));
    return;
  }

  int audioSize = 0;
  while (audioFile.available() && audioSize < maxAudioSize) {
    int16_t sample;
    audioFile.read((uint8_t*)&sample, 2);
    audioData[audioSize++] = sample;
  }

  for (int i = 0; i < n_mfcc; i++) {
    mfcc[i] = 0.0f;
    for (int j = 0; j < audioSize; j++) {
      mfcc[i] += audioData[j] * cos(M_PI * i * (2 * j + 1) / (2 * audioSize));
    }
    mfcc[i] /= audioSize;
  }
}

void scaler(float* features, int size, const char* scalerPath) {
  File scalerFile = SD.open(scalerPath);
  if (!scalerFile) {
    Serial.println(F("Failed to open scaler file"));
    return;
  }

  float mean, std;
  scalerFile.read((uint8_t*)&mean, sizeof(float));
  scalerFile.read((uint8_t*)&std, sizeof(float));
  scalerFile.close();

  for (int i = 0; i < size; i++) {
    features[i] = (features[i] - mean) / std;
  }
}

void differential_mfcc(float* mfcc_features, float* delta_mfccs, float* delta2_mfccs, int size) {
  for (int i = 1; i < size - 1; i++) {
    delta_mfccs[i] = (mfcc_features[i+1] - mfcc_features[i-1]) / 2.0f;
  }

  for (int i = 1; i < size - 1; i++) {
    delta2_mfccs[i] = (delta_mfccs[i+1] - delta_mfccs[i-1]) / 2.0f;
  }
}

void process1(File& audioFile, float* features) {
  float* mfcc = (float*)ps_malloc(40 * sizeof(float));
  float* delta_mfccs = (float*)ps_calloc(40, sizeof(float));
  float* delta2_mfccs = (float*)ps_calloc(40, sizeof(float));
  
  feature_extractor(audioFile, mfcc, 40);
  scaler(mfcc, 40, "/first_model_scaler.pkl");
  differential_mfcc(mfcc, delta_mfccs, delta2_mfccs, 40);

  memcpy(features, mfcc, 40 * sizeof(float));
  memcpy(features + 40, delta_mfccs, 40 * sizeof(float));
  memcpy(features + 80, delta2_mfccs, 40 * sizeof(float));

  free(mfcc);
  free(delta_mfccs);
  free(delta2_mfccs);
}

void process2(File& audioFile, float* features) {
  float* mfcc = (float*)ps_malloc(80 * sizeof(float));
  float* delta_mfccs = (float*)ps_calloc(80, sizeof(float));
  float* delta2_mfccs = (float*)ps_calloc(80, sizeof(float));

  feature_extractor(audioFile, mfcc, 80);
  differential_mfcc(mfcc, delta_mfccs, delta2_mfccs, 80);

  memcpy(features, mfcc, 80 * sizeof(float));
  memcpy(features + 80, delta_mfccs, 80 * sizeof(float));
  memcpy(features + 160, delta2_mfccs, 80 * sizeof(float));

  scaler(features, 240, "/second_model_scaler.pkl");

  free(mfcc);
  free(delta_mfccs);
  free(delta2_mfccs);
}

int model_inference(tflite::MicroInterpreter* interpreter, float* input_data, int input_size) {
  float* input = interpreter->input(0)->data.f;
  memcpy(input, input_data, input_size * sizeof(float));

  interpreter->Invoke();

  float* output = interpreter->output(0)->data.f;
  int output_size = interpreter->output(0)->dims->data[1];
  return std::distance(output, std::max_element(output, output + output_size));
}

void pipeline() {
  File audioFile = SD.open("/audio.wav", FILE_READ);
  if (!audioFile) {
    Serial.println(F("Failed to open audio file"));
    return;
  }
  int pred = model_predict(audioFile, "/converted_first_model.tflite", 120);
  const char* eof_marker = "EOF";
  const char* answer;

  if (pred == -1) {
    Serial.println("error");
    answer = "-1";
  } else if (!pred) {
    Serial.println(F("model1: no pain"));
    model_predict(audioFile, "/converted_second_model.tflite", 120);
    audioFile.seek(0);
    switch(pred) {
      case 0:
        Serial.println(F("model2: awake"));
        answer = "1";
        break;
      case 1:
        Serial.println(F("model2 diaper"));
        answer = "2";
        break;
      case 2:
        Serial.println(F("model2: hug"));
        answer = "3";
        break;
      case 3:
        Serial.println(F("model2: hungry"));
        answer = "4";
        break;
      case 4:
        Serial.println(F("model2: sleepy"));
        answer = "5";
        break;
      default:
        Serial.println(F("model2: wrong result"));
        answer = "-1";
    }
  } else {
    Serial.println(F("pain"));
    answer = "0";
  }
  audioFile.close();
  pCharacteristic->setValue((uint8_t*)answer, strlen(answer));
  pCharacteristic->notify();
  delay(500);
  pCharacteristic->setValue((uint8_t*)eof_marker, strlen(eof_marker));
  pCharacteristic->notify();
  Serial.println("EOF 전송 완료");
}

int model_predict(File& audioFile, char* modelPath, int featureNum) {
  const int kTensorArenaSize = 50 * 1024;
  float* features = (float*)ps_malloc(featureNum * sizeof(float));
  if (!features) {
    Serial.println(F("Failed to allocate features"));
    return -1;
  }
  Serial.println("pass feature");

  uint8_t* tensor_arena = (uint8_t*)ps_malloc(kTensorArenaSize);
  if (!tensor_arena) {
    Serial.println(F("Failed to allocate tensor arena"));
    free(features);
    return -1;
  }
  Serial.println("pass tensor");

  File modelFile = SD.open(modelPath, FILE_READ);
  if (!modelFile) {
    Serial.println(F("Failed to open model files"));
    free(features);
    free(tensor_arena);
    modelFile.close();
    return -1;
  }
  Serial.println(F("pass modelFile"));

  uint8_t* modelData = (uint8_t*)ps_malloc(modelFile.size());

  if (!modelData) {
    Serial.println(F("Failed to allocate memory for models"));
    modelFile.close();
    free(modelData);
    free(features);
    free(tensor_arena);
    return -1;
  }
  Serial.println(F("pass modelData"));

  modelFile.read(modelData, modelFile.size());
  modelFile.close();
  Serial.println(F("pass modelFile Read"));

  Serial.println(F("get model"));
  const tflite::Model* model = tflite::GetModel(modelData);
  Serial.println(F("pass get model"));

  Serial.println(F("interpreter"));
  interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, kTensorArenaSize, &error_reporter);

  Serial.println(F("allocated tensors"));
  (*interpreter).AllocateTensors();
  Serial.println(F("pass allocated tensors"));

  Serial.println(F("Starting audio processing"));
  if (featureNum == 120)
    process1(audioFile, features);
  else
    process2(audioFile, features);
  Serial.println(F("Audio Processing completed"));
  
  int result = model_inference(interpreter, features, featureNum);

  free(features);
  free(tensor_arena);
  free(modelData);
  free(interpreter);

  return result;
}