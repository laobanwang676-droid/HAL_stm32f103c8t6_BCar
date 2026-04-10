#ifndef SOFT_I2C_H
#define SOFT_I2C_H
#include "stm32f1xx_hal.h"

// ===================== 软I2C引脚定义（STM32F103C8T6，PB3=SDA，PB4=SCL） =====================
#define I2C_SDA_PIN    GPIO_PIN_3
#define I2C_SCL_PIN    GPIO_PIN_4
#define I2C_GPIO_PORT  GPIOB

// 引脚电平控制宏
#define I2C_SDA_H()  HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET)   // SDA置高
#define I2C_SDA_L()  HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET) // SDA置低
#define I2C_SCL_H()  HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET)   // SCL置高
#define I2C_SCL_L()  HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET) // SCL置低
// 读取SDA引脚电平（双向IO，接收数据时用）
#define I2C_READ_SDA()  HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN)

void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
uint8_t Soft_I2C_Send_Byte(uint8_t dat);
uint8_t Soft_I2C_Read_Byte(uint8_t ack);
int8_t Soft_I2C_SendBytes(uint8_t Addr, uint8_t *pData, uint16_t Size);
int8_t Soft_I2C_RecvBytes(uint8_t Addr, uint8_t *pData, uint16_t Size);

#endif /*SOFT_I2C_H*/
