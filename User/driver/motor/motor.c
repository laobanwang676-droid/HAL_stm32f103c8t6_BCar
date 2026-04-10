#include "motor.h"
#include "main.h"
/*
    PWM波A : TIM1 CH4 PA11
    PWM波B : TIM1 CH1 PA8
    AN2 (用于控制左电机 正 方向) : PB12    高电平
    AN1 (用于控制左电机 正 方向) : PB13    低电平

    BN1 (用于控制右电机 正 方向) : PB14    低电平
    BN2 (用于控制右电机 正 方向) : PB15    高电平
*/
#define PWM_DUTY_MAX 90         //(0~100)
#define PWM_DUTY_MIN -90        //(-100~0)

extern TIM_HandleTypeDef htim1;

void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 启动TIM1的PWM输出（通道1）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // 启动TIM1的PWM输出（通道4）

}

static void Motor_L_forward(void)//左电机前进
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
}

static void Motor_L_backward(void)//左电机后退
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
}

static void Motor_R_forward(void)//右电机前进
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
}

static void Motor_R_backward(void)//右电机后退
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void Motor_L_speed(float duty)
{
    if(duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    if(duty < PWM_DUTY_MIN) duty = PWM_DUTY_MIN;
    if(duty >= 0)
    {
        Motor_L_forward();
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)(duty / 100.0f * 999));
    }
    else
    {
        Motor_L_backward();
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t)(-duty / 100.0f * 999));
    }
}

void Motor_R_speed(float duty)
{
    if(duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    if(duty < PWM_DUTY_MIN) duty = PWM_DUTY_MIN;
    if(duty >= 0)
    {
        Motor_R_forward();
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(duty / 100.0f * 999));
    }
    else
    {
        Motor_R_backward();
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(-duty / 100.0f * 999));
    }
}

