#include <stdio.h>
#include "mpu6050.h"
#include "soft_i2c.h"
#include "oled.h"
#include "my_math.h"
#include "math.h"

MPU6050_Data_t MPU6050_ReadData;
MPU6050_Angle_t MPU6050_Angle;
MPU6050_Data_t MPU6050_Offset; // 校准得到的平均偏移量（单位与 MPU6050_ReadData 相同）
static uint64_t last_time = 0;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,//过程噪声方差，衡量系统对真实值变化的响应速度，值越大响应越快，但抖动也越大
    .Q_bias = 0.00f,//过程噪声方差，衡量系统对偏差变化的响应速度，值越大响应越快，但抖动也越大
    .R_measure = 0.03f//测量噪声方差，衡量测量值的可信度，值越大表示测量值越不可信，滤波结果更平滑，但响应更慢
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.00f,
    .R_measure = 0.03f,
};

Kalman_t KalmanZ = {
    .Q_angle = 0.001f,//过程噪声方差，衡量系统对真实值变化的响应速度，值越大响应越快，但抖动也越大
    .Q_bias = 0.00f,//过程噪声方差，衡量系统对偏差变化的响应速度，值越大响应越快，但抖动也越大
    .R_measure = 0.03f//测量噪声方差，衡量测量值的可信度，值越大表示测量值越不可信，滤波结果更平滑，但响应更慢
};

static void mpu_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    Soft_I2C_SendBytes(0xD0, buf, 2);
}

static uint8_t mpu_read_reg(uint8_t reg)
{   
    uint8_t data;
    Soft_I2C_SendBytes(0xD0, &reg, 1);
    Soft_I2C_RecvBytes(0xD0, &data, 1);
    return data;
}

// 批量读取连续寄存器，len 最大为 255
static void mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Soft_I2C_SendBytes(0xD0, &reg, 1);
    Soft_I2C_RecvBytes(0xD0, buf, len);
}

void mpu6050_init(void)
{
    mpu_write_reg(0x6b, 0x80);//复位MPU6050
    HAL_Delay(100); // 等待复位完成
    mpu_write_reg(0x6b, 0x00);//唤醒MPU6050
    mpu_write_reg(0x1b, 0x18);//陀螺仪±2000度/秒
    mpu_write_reg(0x1c, 0x00);//加速度计±2g
    mpu_write_reg(0x19, 0x04);//采样率1KHz/(1+4)=200Hz
    mpu_write_reg(0x1a, 0x02);//98Hz带宽
    mpu_write_reg(0x38, 0x00);//关闭中断
    mpu_write_reg(0x6a, 0x00);//I2C主模式关闭
    mpu_write_reg(0x23, 0x00);//不使用FIFO
    mpu_write_reg(0x37, 0x80);//INT引脚低电平有效，推挽输出
    mpu_write_reg(0x6b, 0x01);//设置时钟源为PLL，X轴陀螺仪为参考
    mpu_write_reg(0x6c, 0x00);//所有传感器均工作
}

static void mpu6050_update(void)
{
    int16_t ax_data, ay_data, az_data, gx_data, gy_data, gz_data, temp_data;
    uint8_t buf[14];
    // 一次性读取从 0x3B 开始的 14 字节（ax..az,temp,gx..gz）
    mpu_read_regs(0x3B, buf, 14);
    ax_data = (int16_t)(buf[0] << 8 | buf[1]);
    ay_data = (int16_t)(buf[2] << 8 | buf[3]);
    az_data = (int16_t)(buf[4] << 8 | buf[5]);
    temp_data = (int16_t)(buf[6] << 8 | buf[7]);
    gx_data = (int16_t)(buf[8] << 8 | buf[9]);
    gy_data = (int16_t)(buf[10] << 8 | buf[11]);
    gz_data = (int16_t)(buf[12] << 8 | buf[13]);
    //加速度数据转换为g
    MPU6050_ReadData.ax = (float)(ax_data) * 6.1035e-5f - MPU6050_Offset.ax;
    MPU6050_ReadData.ay = (float)(ay_data) * 6.1035e-5f - MPU6050_Offset.ay;
    MPU6050_ReadData.az = (float)(az_data) * 6.1035e-5f;
    //陀螺仪数据转换为度/秒
    MPU6050_ReadData.gx = (float)(gx_data) * 0.061035f - MPU6050_Offset.gx;
    MPU6050_ReadData.gy = (float)(gy_data) * 0.061035f - MPU6050_Offset.gy;
    MPU6050_ReadData.gz = (float)(gz_data) * 0.061035f - MPU6050_Offset.gz;
    //温度数据转换为摄氏度
    MPU6050_ReadData.temp = (float)(temp_data) * 0.00294117647f + 36.53f;
}

// 上电校准：读取50组原始数据，计算每个通道的平均值并保存为偏移量
// 计算使用与 mpu6050_update 相同的量程换算（但不加入手动调整常数）
// 并对一次当前读取结果应用这些偏移，使初始数据去除静态偏移。
void mpu6050_calibrate(void)
{
    int32_t sx=0, sy=0, sgx=0, sgy=0, sgz=0, st=0;
    for(int i=0;i<50;i++)
    {
        uint8_t b[14];
        mpu_read_regs(0x3B, b, 14);
        int16_t ax = (int16_t)(b[0] << 8 | b[1]);
        int16_t ay = (int16_t)(b[2] << 8 | b[3]);
        int16_t temp = (int16_t)(b[6] << 8 | b[7]);
        int16_t gx = (int16_t)(b[8] << 8 | b[9]);
        int16_t gy = (int16_t)(b[10] << 8 | b[11]);
        int16_t gz = (int16_t)(b[12] << 8 | b[13]);
        sx += ax; sy += ay; sgx += gx; sgy += gy; sgz += gz; st += temp;
        HAL_Delay(5);
    }
    // 换算为与 MPU6050_ReadData 相同的单位（注意：未额外加减手动常数）
    MPU6050_Offset.ax = (float)sx / 50.0f * 6.1035e-5f;
    MPU6050_Offset.ay = (float)sy / 50.0f * 6.1035e-5f;
    MPU6050_Offset.gx = (float)sgx / 50.0f * 0.061035f;
    MPU6050_Offset.gy = (float)sgy / 50.0f * 0.061035f;
    MPU6050_Offset.gz = (float)sgz / 50.0f * 0.061035f;
    MPU6050_Offset.temp = (float)st / 50.0f * 0.00294117647f + 36.53f;
}

//一、使用互补滤波计算欧拉角
void mpu6050_process(void)//函数执行需要时间1.6ms
{
    mpu6050_update();
    uint64_t now_time = HAL_GetTick();
    // 使用浮点秒为单位的 dt 进行积分（tim_get_ms 返回毫秒）
    float dt = 0.005f; // 默认 5ms，防止第一次调用 last_time==0 导致异常
    if (last_time != 0) {
        dt = (now_time - last_time) * 0.001f; // ms -> s
    }
    last_time = now_time;
    MPU6050_Angle.g_yaw += MPU6050_ReadData.gz * dt;
    MPU6050_Angle.g_pitch += MPU6050_ReadData.gx * dt;
    MPU6050_Angle.g_roll += MPU6050_ReadData.gy * dt;
    //使用加速度数据计算俯仰角和横滚角
    MPU6050_Angle.a_yaw = MPU6050_Angle.a_yaw; //加速度计无法计算偏航角
    MPU6050_Angle.a_pitch = qatan2( MPU6050_ReadData.ay, MPU6050_ReadData.az) * 180.0f / 3.14159265f;
    MPU6050_Angle.a_roll = qatan2( - MPU6050_ReadData.ax, MPU6050_ReadData.az) * 180.0f / 3.14159265f;
    //融合陀螺仪和加速度计数据
    MPU6050_Angle.yaw = MPU6050_Angle.g_yaw;
    MPU6050_Angle.pitch = MPU6050_Angle.g_pitch * 0.9f + MPU6050_Angle.a_pitch * 0.1f;
    MPU6050_Angle.roll = MPU6050_Angle.g_roll * 0.9f + MPU6050_Angle.a_roll * 0.1f;
}

/*
二、使用卡尔曼滤波计算欧拉角
*/
static float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
    float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

void angle_kalman(void)//函数执行需要时间1.6ms
{
    mpu6050_update();
    uint64_t now_time = HAL_GetTick();
    // 使用浮点秒为单位的 dt 进行积分（tim_get_ms 返回毫秒）
    float dt = 0.005f; // 默认 5ms，防止第一次调用 last_time==0 导致异常
    if (last_time != 0) {
        dt = (now_time - last_time) * 0.001f; // ms -> s
    }
    last_time = now_time;
    MPU6050_Angle.g_yaw += MPU6050_ReadData.gz * dt;
    MPU6050_Angle.g_pitch += MPU6050_ReadData.gx * dt;
    MPU6050_Angle.g_roll += MPU6050_ReadData.gy * dt;
    //使用加速度数据计算俯仰角和横滚角
    MPU6050_Angle.a_yaw = MPU6050_Angle.a_yaw; //加速度计无法计算偏航角
    MPU6050_Angle.a_pitch = qatan2( MPU6050_ReadData.ay, MPU6050_ReadData.az) * 180.0f / 3.14159265f;
    MPU6050_Angle.a_roll = qatan2( - MPU6050_ReadData.ax, MPU6050_ReadData.az) * 180.0f / 3.14159265f;
    //使用卡尔曼滤波融合陀螺仪和加速度计数据
    //偏航角
    MPU6050_Angle.yaw = MPU6050_Angle.g_yaw; //偏航角不使用卡尔曼滤波，直接使用陀螺仪积分结果
    //俯仰角
    MPU6050_Angle.pitch = Kalman_getAngle(&KalmanY, MPU6050_Angle.a_pitch, MPU6050_ReadData.gx, dt);
    //横滚角
    MPU6050_Angle.roll = Kalman_getAngle(&KalmanX, MPU6050_Angle.a_roll, MPU6050_ReadData.gy, dt);
}
