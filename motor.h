#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

void motor_init(void);
void set_motors(int16_t value_motor_a, int16_t value_motor_b);
void motor_test(void);
void motor_enable(bool enable);

#endif //MOTOR_H
