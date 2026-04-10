#ifndef PID_H
#define PID_H

#include <stdint.h>

float Upright_Control(float target_angle, float angle, float gyro);
float Speed_Control(float target_speed, float speed);
float Turn_Control(float target_turn_angle, float turn_angle, float turn_gyro);

#endif /* PID_H */
