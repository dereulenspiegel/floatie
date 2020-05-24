#include "tilt.hpp"
#include <math.h>
#include "mpu/math.hpp"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif


static const char* TAG = "tilt_sensor";

namespace tilt  {
  TiltSensor::TiltSensor(MPU_t* mpu, gpio_num_t int_gpio) {
    this->mpu_sensor = mpu;
    this->int_pin = int_gpio;
  }

  esp_err_t TiltSensor::configure_gpio_in() {
    esp_err_t err;
    err = gpio_set_direction(this->int_pin, GPIO_MODE_INPUT);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set gpio to input, error=%#X", err);
      this->last_err =  err;
      return err;
    }

    err = gpio_pullup_dis(this->int_pin);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to disable pullup, error=%#X", err);
      this->last_err =  err;
      return err;
    }
    err = gpio_pulldown_en(this->int_pin);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to enable pull down, error=%#X", err);
      this->last_err =  err;
      return err;
    }
    return ESP_OK;
  }

  esp_err_t TiltSensor::initialize() {
    esp_err_t err;
    err = configure_gpio_in();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure gpio in, error=%#X", err);
      this->last_err =  err;
      return err;
    }

    while (esp_err_t err = this->mpu_sensor->testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    err = this->mpu_sensor->initialize();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize MPU, error=%#X", err);
      this->last_err =  err;
      return err;
    }
    err = this->mpu_sensor->setDigitalLowPassFilter(mpud::DLPF_5HZ);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure low pass filter, error=%#X", err);
      this->last_err =  err;
      return err;
    }
    this->mpu_sensor->setSampleRate(17);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set sample rate, error=%#X", err);
      this->last_err =  err;
      return err;
    }
    this->mpu_sensor->setAccelFullScale(mpud::ACCEL_FS_2G);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set accel scale, error=%#X", err);
      this->last_err =  err;
      return err;
    }

    /* mpud::int_config_t int_config;
    int_config.level = mpud::INT_LVL_ACTIVE_HIGH;
    int_config.drive = mpud::INT_DRV_PUSHPULL;
    int_config.mode = mpud::INT_MODE_LATCH;
    int_config.clear = mpud::INT_CLEAR_ANYREAD;
    err = this->mpu_sensor->setInterruptConfig(int_config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure interrupt, error=%#X", err);
      this->last_err =  err;
      return err;
    }

    err = this->mpu_sensor->setInterruptEnabled(mpud::INT_EN_RAWDATA_READY);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to enable interrupt, error=%#X", err);
      this->last_err =  err;
      return err;
    } */
    
    return ESP_OK;
  }

  esp_err_t TiltSensor::last_error() {
    return this->last_err;
  }

  bool TiltSensor::mpu_ready() {
    mpud::int_stat_t state = this->mpu_sensor->getInterruptStatus();
    
    return state && mpud::INT_STAT_RAWDATA_READY == 1;
  }

  esp_err_t TiltSensor::sample_mpu() {

    for (int i=0; i<MEDIANROUNDSMAX; i++) {
      while (!mpu_ready()) {
        // Wait until the mpu has data to read
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      mpud::raw_axes_t accelRaw;
      mpud::float_axes_t accelG;
      this->mpu_sensor->acceleration(&accelRaw);
      accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_2G);
      float tilt = calculate_tilt(accelG.x, accelG.y, accelG.z);
      samples.add(tilt);
    }
    return ESP_OK;
  }

  float TiltSensor::tilt() {
    return samples.getAverage(MEDIANAVGCOUNT);
  }

  float TiltSensor::calculate_tilt(float ax, float ay, float az) {
    float pitch = (atan2(ay, sqrt(ax * ax + az * az))) * 180.0 / M_PI;
    float roll = (atan2(ax, sqrt(ay * ay + az * az))) * 180.0 / M_PI;
    return sqrt(pitch * pitch + roll * roll);
  }
}


