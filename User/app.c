#include <stdio.h>
#include "main.h"
#include "oled.h"
#include "mpu6050.h"
#include "my_math.h"
#include "app.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
/*
@简介:主控程序
*/
//主循环时间控制
#define FALL_CHECK_TIME     5 //倒地检测周期ms
#define OLED_SYNC_TIME      10 // OLED刷新周期ms
#define CONTROL_SYNC_TIME   10 // 控制周期ms
#define DISTANCE_SYNC_TIME  200 // 距离测量周期ms
#define LINE_SYNC_TIME      50 //循迹检测周期ms
//距离测量最大值，单位cm
#define DISTANCE_MAX        17.0f //cm
//倒地角度阈值
#define FALLEN_ANGLE_back   45.0f
#define FALLEN_ANGLE_fore  -45.0f
//速度控制步长和最大值
#define SPEED_STEP          10.0f  //度/s
//速度控制最大值，单位度/s
#define SPEED_MAX           400.0f  
/*循迹*/
#define LINE_SPD_MAX        180.0f  //循迹速度限制   （度/s）
#define KP_LINE             15.0f    // 比例系数
#define KD_LINE             5.0f     // 微分系数

static uint8_t obstacle_flag = 0; //障碍=标志
static uint8_t falldowm_flag = 0; //倒地标志
extern volatile uint8_t flag_speed; //速度控制标志（蓝牙）
extern volatile uint8_t flag_turn;  //转向控制标志（蓝牙）
extern volatile uint8_t flag_stop;  //停止标志（蓝牙）

static uint32_t control_time = 0;//控制周期计时器
static uint32_t oled_time = 0;//OLED刷新周期计时器
static uint32_t distance_time = 0;//距离测量周期计时器
static uint32_t fall_check_time = 0;//倒地检测周期计时器
static uint32_t line_time = 0;//循迹检测周期计时器

//static uint64_t start_time = 0;//测试用

static float distance = 0.0f; //距离测量结果
static float speed = 0.0f;//当前速度
static float PWM_OUT = 0.0f;//输出PWM值
static float PWM_L = 0.0f;//左轮PWM值
static float PWM_R = 0.0f;//右轮PWM值

static float Speed_Control_out;//速度控制输出
static float Upright_Control_out;//自平衡控制输出
static float Turn_Control_out;//转向控制输出
//闭环设定值
static float S_Target_Speed = 0.0f; //速度设定值
static float T_Target_Angle = 0.0f; //转向设定值

//主循环时间控制函数，使用SysTick中断实现
void HAL_SYSTICK_Callback(void)
{
    if(control_time > 0)
        control_time --;
    if(oled_time > 0)
        oled_time --;
    if(distance_time > 0)
        distance_time --;
    if(fall_check_time > 0)
        fall_check_time --;
}

void app_init(void)
{
    oled_init();
    my_math_init();
    mpu6050_init();
    HAL_Delay(3000); //等待稳定
    mpu6050_calibrate();
    motor_init();
    Encoder_Init();
}

//HC-SR04测距函数，返回距离值，单位cm
static float hc_sr04_distance(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay_us(20);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

    uint64_t start_time1 = HAL_GetTick_us();
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
        if(HAL_GetTick_us() - start_time1 > 20000) 
            return 999.9f;//超时，返回一个异常值
    }
    uint64_t start_time2 = HAL_GetTick_us();
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
    {
        if(HAL_GetTick_us() - start_time2 > 20000) 
            return 999.9f;
    }
    uint64_t end_time = HAL_GetTick_us();

    return ((end_time - start_time2) * 0.034f / 2.0f); // cm/s
}

static uint8_t read_line_sensor(void)
{
    uint8_t pin1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) ? 1 : 0;
    uint8_t pin2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) ? 1 : 0;
    uint8_t pin3 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) ? 1 : 0;
    uint8_t pin4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? 1 : 0;
    uint8_t pin5 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) ? 1 : 0;

    return (pin1 << 4) | (pin2 << 3) | (pin3 << 2) | (pin4 << 1) | pin5;
}

static void line_sensor(void)
{
    if(line_time <= 0)
        line_time = LINE_SYNC_TIME;
    else    return;
    static float Line_Error = 0.0f;    // 当前循迹误差
    static float Line_Error_Last = 99.9f;// 上一次的循迹误差 (用于计算D项)
    uint8_t line_state = read_line_sensor();

    switch (line_state) {
        // --- 情况A：在正中间 (误差为0) ---
        case 0b11011: 
        case 0b10001: 
        case 0b10011: 
        case 0b11001: 
            S_Target_Speed += 20.0f; //在正中间时加速
            if(S_Target_Speed > LINE_SPD_MAX)
                S_Target_Speed = LINE_SPD_MAX;
            Line_Error = 0.0f; 
            break;

        // --- 情况B：轻微偏左 (需要向右修正，误差为小正数) ---
        case 0b10111: Line_Error = 1.0f; break; 
        case 0b00011: Line_Error = 1.5f; break; 

        // --- 情况C：中等偏左 (误差为中正数) ---
        case 0b00111: Line_Error = 2.5f; break; 

        // --- 情况D：严重偏左 (快丢线了，误差为大正数) ---
        case 0b01111: Line_Error = 6.0f; break; 

        // --- 情况E：轻微偏右 (误差为小负数) ---
        case 0b11101: Line_Error = -1.0f; break; 
        case 0b11000: Line_Error = -1.5f; break; 

        // --- 情况F：中等偏右 (误差为中负数) ---
        case 0b11100: Line_Error = -2.5f; break; 

        // --- 情况G：严重偏右 (误差为大负数) ---
        case 0b11110: Line_Error = -6.0f; break; 

        case 0b00000:
            Line_Error_Last = Line_Error; 
            break;

        case 0b11111:
            if(Line_Error_Last == 0.0f)
                S_Target_Speed -= 20.0f; //循迹结束停止
            else 
                Line_Error_Last = Line_Error;
             if(S_Target_Speed < 0.0f)
                S_Target_Speed = 0.0f;
            break;
        default:
            break;
    }

    float turn_output = KP_LINE * Line_Error + KD_LINE * (Line_Error - Line_Error_Last);

    // 6. 最终输出 
    T_Target_Angle = turn_output;

    // 7. 更新历史数据
    Line_Error_Last = Line_Error;    
}

//蓝牙控制函数，根据标志位调整速度和转向设定值
static void command_control(void)
{
    if(flag_stop == 1)
    {
        if(S_Target_Speed < 0.0f)
        {
            S_Target_Speed += 10.0f; 
            if(S_Target_Speed >= 0.0f)
            {
                S_Target_Speed = 0.0f;
                flag_stop = 0;
            }
        }
        else
        {
            S_Target_Speed -= 10.0f;
            if(S_Target_Speed <= 0.0f)
            {
                S_Target_Speed = 0.0f;
                flag_stop = 0;
            }
        }
        flag_speed = 0;
        flag_turn = 0;
        return;
    }

    switch(flag_speed)
    {
        case 1: 
            S_Target_Speed += SPEED_STEP;
            if(S_Target_Speed > SPEED_MAX)
                S_Target_Speed = SPEED_MAX;
            break; 
        case 2: 
            S_Target_Speed -= SPEED_STEP;
            if(S_Target_Speed < -SPEED_MAX)
                S_Target_Speed = -SPEED_MAX;
            break;
        default:
            break;
        }
    switch(flag_turn)
    {
        case 1: 
            T_Target_Angle += 45.0f;
            flag_turn = 0;
            break;
        case 2: 
            T_Target_Angle -= 45.0f;
            flag_turn = 0;
            break;
        default:
            break;
    }
}

//倒地检测函数，根据MPU6050的pitch角度判断是否倒地
static void fall_check(void)
{
    if(fall_check_time <= 0)
        fall_check_time = FALL_CHECK_TIME;
    else    return;
    if(MPU6050_Angle.pitch > FALLEN_ANGLE_back || MPU6050_Angle.pitch < FALLEN_ANGLE_fore)
    {
        //关闭电机输出
        Motor_L_speed(0);
        Motor_R_speed(0);
        //重置设定值
        S_Target_Speed = 0.0f;
        T_Target_Angle = 0.0f;
        //设置倒地标志
        falldowm_flag = 1;
    }
    else
    {
        falldowm_flag = 0;
    }
}

static void control(void)//自平衡控制主函数
{   
    if(control_time <= 0)
        control_time = CONTROL_SYNC_TIME;
    else    return;
    //1使用卡尔曼滤波融合陀螺仪和加速度计数据，得到更准确的姿态角
    angle_kalman();
    // mpu6050_process();
    if(falldowm_flag == 1)
        return;
    //2获取当前速度，使用左右轮编码器的平均值作为当前速度的估计
    Encoder_Get_Speed();
    speed = (speed_L + speed_R) / 2.0f;//当前速度估计取平均值
    //蓝牙控制
    command_control();
    //3速度环输出
    Speed_Control_out = Speed_Control(S_Target_Speed, speed);
    //4直立环输出
    Upright_Control_out = Upright_Control(Speed_Control_out, MPU6050_Angle.pitch, MPU6050_ReadData.gx);
    //5转向环输出
    Turn_Control_out = Turn_Control(T_Target_Angle, MPU6050_Angle.yaw, MPU6050_ReadData.gz);
    //6计算左右轮PWM值
    PWM_OUT = Upright_Control_out;
    PWM_L = PWM_OUT + Turn_Control_out;
    PWM_R = PWM_OUT - Turn_Control_out;
    //7输出PWM值到电机
    Motor_L_speed(PWM_L);
    Motor_R_speed(PWM_R);
}

static void mpu6050_oled(void)//显示MPU6050数据到OLED
{
    char buf[30];
    sprintf(buf, "yaw: %.2f", MPU6050_Angle.yaw);
    oled_write_string(0, 0, buf, 8);
    sprintf(buf, "pitch: %.2f", MPU6050_Angle.pitch);
    oled_write_string(0, 2, buf, 8);
    sprintf(buf, "roll: %.2f",  MPU6050_Angle.roll);
    oled_write_string(0, 4, buf, 8);
}

static void MPU6050_oled_show(void)
{
    if(oled_time <= 0)
        oled_time = OLED_SYNC_TIME;
    else    return;
    mpu6050_oled();
}

static void distance_measure(void)
{
    if(distance_time <= 0)
        distance_time = DISTANCE_SYNC_TIME;
    else    return;

    distance = hc_sr04_distance(); 
    if(distance <= DISTANCE_MAX && obstacle_flag == 0)
    {
        T_Target_Angle += 90.0f; //转向避障
        obstacle_flag = 1;//左转标志
    }
    else if(distance <= DISTANCE_MAX && obstacle_flag == 1)
    {
        T_Target_Angle -= 90.0f; 
        obstacle_flag = 2;//右转标志
    }
    else if(distance <= DISTANCE_MAX && obstacle_flag == 2)
    {
        T_Target_Angle -= 90.0f;
        obstacle_flag = 3;//继续右转标志
    }
    else if(distance <= DISTANCE_MAX && obstacle_flag == 3)
    {
        T_Target_Angle -= 90.0f;
        obstacle_flag = 4;//继续右转标志（原本的后方向）
    }
    else if(distance > DISTANCE_MAX && obstacle_flag == 4)
    {
        S_Target_Speed = 0.0f;
        obstacle_flag = 0;//重置标志，恢复正常行驶
    }
}

void app_loop(void)
{
    fall_check();
    control();
    distance_measure();
    line_sensor();
    MPU6050_oled_show();
}
