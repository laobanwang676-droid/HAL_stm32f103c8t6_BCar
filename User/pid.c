#include "pid.h"
#include "main.h"
#include <stdint.h>
/*
设计三个PID控制器：直立环，速度环，转向环 
*/
#define OUT_MAX 1500.0f        //输出限幅(-1500 ~ 1050)
#define INTEG_MAX 1500.0f     //积分限幅(-1500 ~ 1050)

float upright_kp = 6.5f;      //直立环比例系数(0.5~10.0之间调试) 6.5
float upright_kd = 0.2f;      //直立环微分系数(0.0~5.0之间调试) 0.2

float speed_kp = 0.017f;      //速度环系数(0.0~0.2之间调试) 0.017
//  speed_ki = 1/200 * speed_kp

float turn_kp = 0.7f;//转向环系数 0.7
float turn_kd = 0.025f; //0.025
extern uint8_t flag_stop; //停止标志位

//设置直立环控制函数
//传入参数为 1、期望角度 2、当前角度 3、当前角速度（规定后仰时角度和角速度都为正）
//返回 1、直立环输出
float Upright_Control(float target_angle, float angle, float gyro)
{
    float error = target_angle - angle;//计算角度误差
    float output = upright_kp * error + upright_kd * (-gyro);
    return output;
}

//设置速度环控制函数
//传入参数为 1、期望速度 2、当前速度 
//返回 1、速度环输出
float Speed_Control(float target_speed, float speed)
{
    static uint64_t last_time = 0;
    uint64_t now_time = HAL_GetTick();
    float dt = 0.005f; // 默认 5ms，防止第一次调用 last_time==0 导致异常
    if (last_time != 0) {
        dt = (now_time - last_time) * 0.001f; // ms -> s
        if(dt >= 0.03f) dt = 0.03f; //防止dt过大导致积分过多 
    }
    last_time = now_time;
    static float speed_error_i = 0;//速度误差积分值
    float error = target_speed - speed;//计算速度误差
    speed_error_i += error * dt;
    //积分限幅 防止积分过大
    if (speed_error_i > INTEG_MAX)
        speed_error_i = INTEG_MAX;
    else if (speed_error_i < -INTEG_MAX)
        speed_error_i = -INTEG_MAX;
    float output = - speed_kp * error - (speed_kp * 0.005f) * speed_error_i;
    //输出限幅
    if (output > OUT_MAX)
        output = OUT_MAX;
    else if (output < -OUT_MAX)
        output = -OUT_MAX;
     if(flag_stop == 1) 
        speed_error_i = 0;//如果停止 则清零积分
    return output;
}

//设置转向环控制函数
//传入参数为 1、期望转向角度 2、当前Z轴角度（左转为正）3、当前Z轴角速度
//返回 1、转向环输出
float Turn_Control(float target_turn_angle, float turn_angle, float turn_gyro)
{
    float error = target_turn_angle - turn_angle;//计算转向角度误差
    float output = - turn_kp * error + turn_kd * turn_gyro;//因为左转时角度和角速度都为正 需要取反
    return output;
}
