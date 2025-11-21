#include "odometry_task.h"
#include "accel_driver.h"
#include "esp_timer.h"
#include <math.h>

#define ODOMETRY_TASK_PERIOD_S 0.01f
#define CONV_M_TO_CM 100.0f
#define ACCEL_NOISE_THRESHOLD_MS2 0.1f

static struct {
    struct accel_driver_values_ms accel_values_offset;
    struct odometry current_position;
    float vx_ms;
    float vy_ms;
    float vz_ms;

    float gx_offset;  // giroscópio offsets
    float gy_offset;
    float gz_offset;
} self = {
    .accel_values_offset = {},
    .current_position = {},
    .vx_ms = 0.0f,
    .vy_ms = 0.0f,
    .vz_ms = 0.0f,
    .gx_offset = 0.0f,
    .gy_offset = 0.0f,
    .gz_offset = 0.0f,
};

static void delay_ms(int ms)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (ms * 1000)) { }
}

struct accel_driver_values_ms odometry_offsets_calibration(void)
{
    struct accel_driver_values_ms values = {0};
    struct accel_driver_values_ms sum = {0};
    const int num_samples = 100;

    for (int i = 0; i < num_samples; i++) {
        accel_driver_get_values(&values);

        sum.ax_ms2 += values.ax_ms2;
        sum.ay_ms2 += values.ay_ms2;
        sum.az_ms2 += values.az_ms2;

        self.gx_offset += values.gx_rad;
        self.gy_offset += values.gy_rad;
        self.gz_offset += values.gz_rad;

        delay_ms(10);
    }

    self.accel_values_offset.ax_ms2 = sum.ax_ms2 / num_samples;
    self.accel_values_offset.ay_ms2 = sum.ay_ms2 / num_samples;
    self.accel_values_offset.az_ms2 = sum.az_ms2 / num_samples;

    self.gx_offset /= num_samples;
    self.gy_offset /= num_samples;
    self.gz_offset /= num_samples;

    self.vx_ms = self.vy_ms = self.vz_ms = 0.0f;

    self.current_position.px_cm = 0.0f;
    self.current_position.py_cm = 0.0f;
    self.current_position.pz_cm = 0.0f;

    self.current_position.roll_rad  = 0.0f;
    self.current_position.pitch_rad = 0.0f;
    self.current_position.yaw_rad   = 0.0f;

    return self.accel_values_offset;
}

void odometry_task(void)
{
    struct accel_driver_values_ms raw = {0};
    accel_driver_get_values(&raw);

    struct accel_driver_values_ms linear = {0};
    linear.ax_ms2 = raw.ax_ms2 - self.accel_values_offset.ax_ms2;
    linear.ay_ms2 = raw.ay_ms2 - self.accel_values_offset.ay_ms2;
    linear.az_ms2 = raw.az_ms2 - self.accel_values_offset.az_ms2;

    if (fabsf(linear.ax_ms2) < ACCEL_NOISE_THRESHOLD_MS2) { linear.ax_ms2 = 0; self.vx_ms = 0; }
    if (fabsf(linear.ay_ms2) < ACCEL_NOISE_THRESHOLD_MS2) { linear.ay_ms2 = 0; self.vy_ms = 0; }
    if (fabsf(linear.az_ms2) < ACCEL_NOISE_THRESHOLD_MS2) { linear.az_ms2 = 0; self.vz_ms = 0; }

    self.vx_ms += linear.ax_ms2 * ODOMETRY_TASK_PERIOD_S;
    self.vy_ms += linear.ay_ms2 * ODOMETRY_TASK_PERIOD_S;
    self.vz_ms += linear.az_ms2 * ODOMETRY_TASK_PERIOD_S;

    self.current_position.px_cm += self.vx_ms * ODOMETRY_TASK_PERIOD_S * CONV_M_TO_CM;
    self.current_position.py_cm += self.vy_ms * ODOMETRY_TASK_PERIOD_S * CONV_M_TO_CM;
    self.current_position.pz_cm += self.vz_ms * ODOMETRY_TASK_PERIOD_S * CONV_M_TO_CM;

    float gx = raw.gx_rad - self.gx_offset;
    float gy = raw.gy_rad - self.gy_offset;
    float gz = raw.gz_rad - self.gz_offset;

    self.current_position.roll_rad  += gx * ODOMETRY_TASK_PERIOD_S;
    self.current_position.pitch_rad += gy * ODOMETRY_TASK_PERIOD_S;
    self.current_position.yaw_rad   += gz * ODOMETRY_TASK_PERIOD_S;

    if (self.current_position.yaw_rad > M_PI)
        self.current_position.yaw_rad -= 2 * M_PI;
    if (self.current_position.yaw_rad < -M_PI)
        self.current_position.yaw_rad += 2 * M_PI;
}

struct odometry odometry_get_position(void)
{
    return self.current_position;
}
