#ifndef ACCEL_DRIVER_H
#define ACCEL_DRIVER_H

#include "esp_err.h"

struct accel_driver_values_ms {
    float ax_ms2;
    float ay_ms2;
    float az_ms2;
};

esp_err_t accel_driver_init(void);
esp_err_t accel_driver_setup_mpu6050(void);
esp_err_t accel_driver_get_values(struct accel_driver_values_ms* values);

#endif /* ACCEL_DRIVER_H */