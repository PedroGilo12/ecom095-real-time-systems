#include <stdio.h>

#include "accel_driver.h"

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

#define ACCEL_DRIVER_I2C_MASTER_SCL_IO 1
#define ACCEL_DRIVER_I2C_MASTER_SDA_IO 2
#define ACCEL_DRIVER_I2C_MASTER_FREQ_HZ 100000
#define ACCEL_DRIVER_I2C_MASTER_PORT I2C_NUM_0
#define ACCEL_DRIVER_MPU6050_I2C_ADDR 0x68

#define ACCEL_DRIVER_RAW_TO_MS2(value) ( (value) / 16384.0f * 9.80665f )

#define GYRO_RAW_TO_RADS(value) ( ((value) / 131.0f) * (3.14159265358979f / 180.0f) )

static const char* TAG = "ACCEL_DRIVER";

static esp_err_t accel_driver_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(ACCEL_DRIVER_I2C_MASTER_PORT,
                                      ACCEL_DRIVER_MPU6050_I2C_ADDR,
                                      buf, 2,
                                      1000 / portTICK_PERIOD_MS);
}

static esp_err_t accel_driver_read_bytes(uint8_t reg, uint8_t* data, size_t len)
{
    return i2c_master_write_read_device(ACCEL_DRIVER_I2C_MASTER_PORT,
                                        ACCEL_DRIVER_MPU6050_I2C_ADDR,
                                        &reg, 1,
                                        data, len,
                                        1000 / portTICK_PERIOD_MS);
}

esp_err_t accel_driver_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = ACCEL_DRIVER_I2C_MASTER_SDA_IO,
        .scl_io_num = ACCEL_DRIVER_I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = ACCEL_DRIVER_I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(ACCEL_DRIVER_I2C_MASTER_PORT, &conf));
    return i2c_driver_install(ACCEL_DRIVER_I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

esp_err_t accel_driver_setup_mpu6050(void)
{
    // Wake up MPU6050
    return accel_driver_write_byte(0x6B, 0x00);
}

esp_err_t accel_driver_get_values(struct accel_driver_values_ms* values)
{
    esp_err_t err = ESP_OK;

    uint8_t accel_raw[6];
    uint8_t gyro_raw[6];

    err = accel_driver_read_bytes(0x3B, accel_raw, 6);
    if (err != ESP_OK) return err;

    int16_t ax = (accel_raw[0] << 8) | accel_raw[1];
    int16_t ay = (accel_raw[2] << 8) | accel_raw[3];
    int16_t az = (accel_raw[4] << 8) | accel_raw[5];

    values->ax_ms2 = ACCEL_DRIVER_RAW_TO_MS2(ax);
    values->ay_ms2 = ACCEL_DRIVER_RAW_TO_MS2(ay);
    values->az_ms2 = ACCEL_DRIVER_RAW_TO_MS2(az);

    err = accel_driver_read_bytes(0x43, gyro_raw, 6);
    if (err != ESP_OK) return err;

    int16_t gx = (gyro_raw[0] << 8) | gyro_raw[1];
    int16_t gy = (gyro_raw[2] << 8) | gyro_raw[3];
    int16_t gz = (gyro_raw[4] << 8) | gyro_raw[5];

    values->gx_rad = GYRO_RAW_TO_RADS(gx);
    values->gy_rad = GYRO_RAW_TO_RADS(gy);
    values->gz_rad = GYRO_RAW_TO_RADS(gz);

    return ESP_OK;
}
