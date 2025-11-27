#include "fsm_states.h"

#include <math.h>
#include <stdio.h>

#include "motor_driver.h"
#include "odometry_task.h"

#include "driver/gpio.h" 
#include "esp_timer.h"

#define ODOMETRY_TASK_PERIOD_MS 2
#define VEL_SECONDS_PER_CM 35.7
#define DISTANCE 10
#define PRINT_PERIOD_MS (DISTANCE * VEL_SECONDS_PER_CM)

#define KP_YAW 28.0f
#define KI_YAW 0.90f
#define KD_YAW 0.00f

#define YAW_TARGET_DEG 0.0f
#define BASE_SPEED 180
#define PID_OUTPUT_MAX 255.0f
#define PID_OUTPUT_MIN -255.0f

#define INTEGRAL_MAX 200.0f
#define INTEGRAL_MIN -200.0f

static float integral_term = 0.0f;
static float last_error    = 0.0f;
volatile int last_int = 0;

static float rad_to_deg(float rad)
{
    return rad * 180.0f / (float) M_PI;
}

static float normalize_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static void delay_ms(int ms)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (ms * 1000)) {}
}


int fsm_state_waiting_goal(struct fsm_event *event)
{
    switch (event->id) {
    case FSM_EVENT_ID_ENTRY: {
        motor_driver_move_left(0, false);
        motor_driver_move_right(0, false);
    } break;
    case FSM_EVENT_ID_RIGHT_BUMPER: {
        uint8_t next_state = FSM_STATE_MOTION_TO_GOAL;
        fsm_event_post(FSM_EVENT_ID_EXIT, &next_state, sizeof(next_state));
    } break;
    default: {
    } break;
    }
    return 0;
}

int fsm_state_motion_to_goal(struct fsm_event *event)
{
    switch (event->id) {
    case FSM_EVENT_ID_RUN_TASK: {
        struct odometry pos = odometry_get_position();
        float yaw_deg       = rad_to_deg(pos.yaw_rad);
        float error         = normalize_angle(YAW_TARGET_DEG - yaw_deg);

        float P = KP_YAW * error;

        float dt = (ODOMETRY_TASK_PERIOD_MS / 1000.0f);
        integral_term += error * dt * KI_YAW;
        if (integral_term > INTEGRAL_MAX)
            integral_term = INTEGRAL_MAX;
        if (integral_term < INTEGRAL_MIN)
            integral_term = INTEGRAL_MIN;
        float I = integral_term;

        float derivative = (error - last_error) / dt;
        float D          = KD_YAW * derivative;

        last_error = error;

        float control = P + I + D;
        if (control > PID_OUTPUT_MAX)
            control = PID_OUTPUT_MAX;
        if (control < PID_OUTPUT_MIN)
            control = PID_OUTPUT_MIN;

        int left_speed_raw  = BASE_SPEED - (int) round(control);
        int right_speed_raw = BASE_SPEED + (int) round(control);

        int left_speed  = abs(left_speed_raw);
        int right_speed = abs(right_speed_raw);

        if (left_speed > 255)
            left_speed = 255;
        if (right_speed > 255)
            right_speed = 255;

        bool left_fwd  = left_speed_raw >= 0;
        bool right_fwd = right_speed_raw >= 0;

        motor_driver_move_left(left_speed, left_fwd);
        motor_driver_move_right(right_speed, right_fwd);

    } break;

    case FSM_EVENT_ID_RIGHT_BUMPER: {
        last_int           = 4;
        uint8_t next_state = FSM_STATE_BOUNDARY_FOLLOWING;
        fsm_event_post(FSM_EVENT_ID_EXIT, &next_state, sizeof(next_state));
    } break;

    case FSM_EVENT_ID_LEFT_BUMPER: {
        last_int           = 3;
        uint8_t next_state = FSM_STATE_BOUNDARY_FOLLOWING;
        fsm_event_post(FSM_EVENT_ID_EXIT, &next_state, sizeof(next_state));
    } break;

    default: {
        /* Do nothing */
    } break;
    }
    return 0;
}

int fsm_state_boundary_following(struct fsm_event *event)
{
    switch (event->id) {
    case FSM_EVENT_ID_ENTRY: {
    } break;
    case FSM_EVENT_ID_RUN_TASK: {
        if (last_int == 3) {
            if (gpio_get_level(3) == 1) {
                last_int = 0;
            }
        } else if (last_int == 4) {
            if (gpio_get_level(4) == 1) {
                last_int = 0;
            }
        }

        if (last_int == 3) {
            motor_driver_move_right(255, true);
            motor_driver_move_left(255, false);
        } else if (last_int == 4) {
            motor_driver_move_left(255, true);
            motor_driver_move_right(255, false);
        } else {
            uint8_t next_state = FSM_STATE_MOTION_TO_GOAL;
            fsm_event_post(FSM_EVENT_ID_EXIT, &next_state, sizeof(next_state));
        }
    }break;
    case FSM_EVENT_ID_EXIT: {
    } break;
    default: {
    } break;
    }
    return 0;
}
