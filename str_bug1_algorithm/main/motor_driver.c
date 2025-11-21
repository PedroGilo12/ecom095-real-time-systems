#include "motor_driver.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define MOTOR_DRIVER_GPIO_RIGHT_A  18
#define MOTOR_DRIVER_GPIO_RIGHT_B 15
#define MOTOR_DRIVER_GPIO_LEFT_A  9
#define MOTOR_DRIVER_GPIO_LEFT_B  14

#define MOTOR_PWM_FREQ     5000
#define MOTOR_PWM_RES      LEDC_TIMER_8_BIT

static void motor_set_pwm(uint8_t value, int gpio)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, gpio, value);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, gpio);
}

void motor_driver_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQ,
        .duty_resolution = MOTOR_PWM_RES,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channels[4] = {
        {
            .channel = LEDC_CHANNEL_0,
            .gpio_num = MOTOR_DRIVER_GPIO_RIGHT_A,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0
        },
        {
            .channel = LEDC_CHANNEL_1,
            .gpio_num = MOTOR_DRIVER_GPIO_RIGHT_B,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0
        },
        {
            .channel = LEDC_CHANNEL_2,
            .gpio_num = MOTOR_DRIVER_GPIO_LEFT_A,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0
        },
        {
            .channel = LEDC_CHANNEL_3,
            .gpio_num = MOTOR_DRIVER_GPIO_LEFT_B,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0
        }
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&channels[i]);
    }
}

void motor_driver_move_right(uint8_t value, bool direction)
{
    if (direction) {
        motor_set_pwm(value, LEDC_CHANNEL_0);
        motor_set_pwm(0,     LEDC_CHANNEL_1);
    } else {
        motor_set_pwm(0,     LEDC_CHANNEL_0);
        motor_set_pwm(value, LEDC_CHANNEL_1);
    }
}

void motor_driver_move_left(uint8_t value, bool direction)
{
    if (direction) {
        motor_set_pwm(value, LEDC_CHANNEL_2);
        motor_set_pwm(0,     LEDC_CHANNEL_3);
    } else {
        motor_set_pwm(0,     LEDC_CHANNEL_2);
        motor_set_pwm(value, LEDC_CHANNEL_3);
    }
}
