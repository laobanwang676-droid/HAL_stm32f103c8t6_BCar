#ifndef MY_MATH_H
#define MY_MATH_H

#include <stdint.h>

// 快速三角函数声明（查表实现，输入为弧度值）
float qsin(float x);
float qcos(float x);
float qtan(float x);
float qasin(float x);
float qacos(float x);
float qatan(float x);
float qatan2(float y, float x);

// 三角函数查表初始化函数（需在main的while(1)前调用一次）
void my_math_init(void);

#endif /* MY_MATH_H */
