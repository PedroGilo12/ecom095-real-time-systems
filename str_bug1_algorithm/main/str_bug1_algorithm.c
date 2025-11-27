#include <stdio.h>
#include "accel_driver.h"
#include "motor_driver.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "fsm.h"
#include "odometry_task.h"

#define RIGHT_BUMPER 3
#define LEFT_BUMPER  4

#define DEBOUNCE_US 5000 

static const char* TAG = "APP_MAIN";

static volatile int64_t last_left_isr_time = 0;
static volatile int64_t last_right_isr_time = 0;
static int64_t last_wake_time = 0;

static void delay_ms(int ms)
{
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (ms * 1000)) {}
}

static void periodic_delay_ms(int period_ms)
{
    int64_t now = esp_timer_get_time();

    if (last_wake_time == 0) {
        last_wake_time = now;
    }

    int64_t next_wake = last_wake_time + (period_ms * 1000);
    int64_t remaining = next_wake - now;

    if (remaining > 0) {
        while ((esp_timer_get_time() - now) < remaining) {
        }
    }

    last_wake_time = next_wake;
}

static void IRAM_ATTR left_bumper_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - last_left_isr_time > DEBOUNCE_US) {
        last_left_isr_time = now;
        fsm_event_post(FSM_EVENT_ID_LEFT_BUMPER, NULL, 0);
    }
}

static void IRAM_ATTR right_bumper_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - last_right_isr_time > DEBOUNCE_US) {
        last_right_isr_time = now;
        fsm_event_post(FSM_EVENT_ID_RIGHT_BUMPER, NULL, 0);
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando I2C");
    ESP_ERROR_CHECK(accel_driver_init());
    ESP_LOGI(TAG, "Configurando MPU6050");
    ESP_ERROR_CHECK(accel_driver_setup_mpu6050());
    motor_driver_init();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RIGHT_BUMPER) | (1ULL << LEFT_BUMPER),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(RIGHT_BUMPER, left_bumper_isr, NULL);
    gpio_isr_handler_add(LEFT_BUMPER, right_bumper_isr, NULL);

    delay_ms(100);

    int counter = 1;

    while (1) {

        periodic_delay_ms(6);

        switch(counter) {
            case 1: {
                /* Accel Polling*/
                accel_driver_task();

                /* Odometry */
                odometry_task();

            } break; 
            case 2: {

                /* Accel Polling*/
                accel_driver_task();
                
                /* Odometry */
                odometry_task();

                /* Bug1 Algorithm */
                fsm_event_post(FSM_EVENT_ID_RUN_TASK, NULL, 0);
                fsm_state_task();

            } break;
            default: {
            } break;
        }

        counter++;
        if(counter == 3) {
            counter = 1;
        }
    }
}
