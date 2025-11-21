#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

void motor_driver_init(void);

void motor_driver_move_right(uint8_t value, bool direction);

void motor_driver_move_left(uint8_t value, bool direction);

#endif /* MOTOR_DRIVER_H */