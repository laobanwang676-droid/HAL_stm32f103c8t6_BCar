/*
编码器设置
    左电机A相： TIM2 CH2 PA1
    左电机B相： TIM2 CH1 PA0
    右电机A相： TIM4 CH2 PB7
    右电机B相： TIM4 CH1 PB6
*/
#include "main.h"
#include "encoder.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;
static float last_time = 0;
static int32_t total_count_L = 0;
static uint16_t last_counter_L = 0;
static int32_t total_count_R = 0;
static uint16_t last_counter_R = 0;
float speed_L = 0.0f;
float speed_R = 0.0f;

// 新增：三次采样的缓存数组
#define SAMPLE_CNT 3  // 三次采样
/* 一阶低通滤波系数（α=0.5，值越大越实时，越小越平滑；范围0.1~0.5） */
#define LPF_ALPHA        0.3f
// 新增全局变量：限幅滤波和低通滤波的缓存
static float speed_L_lpf = 0.0f;        // 左电机低通后速度
static float speed_R_lpf = 0.0f;        // 右电机低通后速度

static int16_t delta_L_buf[SAMPLE_CNT] = {0};  // 左电机增量缓存
static int16_t delta_R_buf[SAMPLE_CNT] = {0};  // 右电机增量缓存
static uint8_t sample_idx = 0;                 // 采样索引

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

float Encoder_Get_L(void) // 取为正方向
{
    uint16_t current = __HAL_TIM_GetCounter(&htim2);
    int16_t delta = (int16_t)(current - last_counter_L);  // 计算增量，处理溢出
    total_count_L += delta;
    last_counter_L = current;
    return total_count_L / 30.0f / 44.0f * 360.0f;// 转换为角度
}

float Encoder_Get_R(void) // 需要取反 因为电机摆放位置对称
{
    uint16_t current = __HAL_TIM_GetCounter(&htim4);
    int16_t delta = (int16_t)(current - last_counter_R);  // 计算增量，处理溢出
    total_count_R += delta;
    last_counter_R = current;
    return (-total_count_R / 30.0f / 44.0f * 360.0f);// 转换为角度
}

void Encoder_Get_Speed(void)
{
    float now_time = (float)HAL_GetTick(); // 获取当前时间（单位：ms）
    for(; sample_idx < SAMPLE_CNT; sample_idx++)
    {
        // 1. 读取当前计数器，计算单次增量
        uint16_t current_L = __HAL_TIM_GetCounter(&htim2);
        int16_t delta_L = (int16_t)(current_L - last_counter_L);
        total_count_L += delta_L;
        last_counter_L = current_L;

        uint16_t current_R = __HAL_TIM_GetCounter(&htim4);
        int16_t delta_R = (int16_t)(current_R - last_counter_R);
        total_count_R += delta_R;
        last_counter_R = current_R;

        // 2. 将单次增量存入缓存
        delta_L_buf[sample_idx] = delta_L;
        delta_R_buf[sample_idx] = delta_R;
        HAL_Delay(1); // 1ms 采样间隔
    }
    float dt = (now_time - last_time) / SAMPLE_CNT; // 平均时间间隔
    last_time = now_time;

    sample_idx = 0; // 重置索引
    // 计算三次增量的总和
    int32_t sum_delta_L = 0;
    int32_t sum_delta_R = 0;
    for(uint8_t i=0; i<SAMPLE_CNT; i++)
    {
        sum_delta_L += delta_L_buf[i];
        sum_delta_R += delta_R_buf[i];
    }
    // 计算平均增量（三次采样平均）
    float avg_delta_L = (float)sum_delta_L / 3.0f;
    float avg_delta_R = (float)sum_delta_R / 3.0f;

    // 4. 基于平均增量计算速度（单位：度/秒）
    float raw_speed_L = avg_delta_L / 30.0f / 44.0f * 360.0f / (dt / 1000.0f ); 
    float raw_speed_R = -avg_delta_R / 30.0f / 44.0f * 360.0f / (dt / 1000.0f ); 

    // ========== 一阶低通滤波（平滑小毛刺） ==========
    speed_L_lpf = LPF_ALPHA * raw_speed_L + (1 - LPF_ALPHA) * speed_L_lpf;
    speed_R_lpf = LPF_ALPHA * raw_speed_R + (1 - LPF_ALPHA) * speed_R_lpf;

    // 替换原有速度值（后续用滤波后的speed_L_lpf/speed_R_lpf）
    speed_L = speed_L_lpf;
    speed_R = speed_R_lpf;    
}

