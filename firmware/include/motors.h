#pragma once

void motors_init(void);
void set_motor(int idx, int pwm, int dir);
void motors_stop_all(void);
