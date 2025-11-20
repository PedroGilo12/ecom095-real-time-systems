#include <stdio.h>
#include "accel_driver.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

static const char* TAG = "APP_MAIN";

static void delay_ms(int ms)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (ms * 1000)) {
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando I2C");
    ESP_ERROR_CHECK(accel_driver_init());

    ESP_LOGI(TAG, "Configurando MPU6050");
    ESP_ERROR_CHECK(accel_driver_setup_mpu6050());

    delay_ms(100);

    while (1) {
        struct accel_driver_values_ms values = {0};

        accel_driver_get_values(&values);
        printf("Acel m/s2: X=%.3f  Y=%.3f  Z=%.3f\n", values.ax_ms2, values.ay_ms2, values.az_ms2);

        delay_ms(500);
    }
}
