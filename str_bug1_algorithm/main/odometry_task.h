#ifndef ODOMETRY_TASK
#define ODOMETRY_TASK

struct odometry {
    float px_cm;
    float py_cm;
    float pz_cm;

    float roll_rad;
    float pitch_rad;
    float yaw_rad;
};
void odometry_task(void);

struct accel_driver_values_ms odometry_offsets_calibration(void);

struct odometry odometry_get_position(void);

#endif /* ODOMETRY_TASK */