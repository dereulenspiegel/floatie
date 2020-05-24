#ifndef TILT_H
#define TILT_H

#include "esp_err.h"
#include "driver/gpio.h"

#include "MPU.hpp"
#include "mpu/types.hpp"
#include "RunningMedian.h"
#include "sdkconfig.h"

#define MEDIANROUNDSMAX CONFIG_TILT_MEDIANROUNDSMAX

#define MEDIANAVGCOUNT CONFIG_TILT_MEDIANAVGCOUNT

#ifndef MEDIANROUNDSMAX
  #define MEDIANROUNDSMAX 10
#endif

namespace tilt {
#ifdef __cplusplus
extern "C"
{
#endif

float calculate_tilt(float ax, float ay, float az);

// esp_err_t sample_accelerometer(MPU_t* device, mpud::float_axes_t* out);

#ifdef __cplusplus
}
#endif

  class TiltSensor {
    public:
      TiltSensor(MPU_t* mpu_sensor, gpio_num_t int_gpio);
      esp_err_t initialize();
      float tilt();
      esp_err_t sample_mpu();
      esp_err_t last_error();

    protected:
      esp_err_t last_err;
      MPU_t* mpu_sensor;
      gpio_num_t int_pin;
      RunningMedian samples = RunningMedian(MEDIANROUNDSMAX);

      float calculate_tilt(float ax, float ay, float az);
      bool mpu_ready();
      esp_err_t configure_gpio_in();
  };
}

typedef tilt::TiltSensor tilt_sensor_t;

#endif