#include "main.h"
#include "soft_i2c.h"

static void I2C_Delay_us(uint32_t us)
{
    HAL_Delay_us(us);
}

/**
 * @brief  I2C起始信号（SDA设为输出）
 */
void Soft_I2C_Start(void)
{
    I2C_SDA_L();
    I2C_Delay_us(1);
}

/**
 * @brief  I2C停止信号（SCL高时，SDA从低→高）
 */
void Soft_I2C_Stop(void)
{
    I2C_SCL_L();
    I2C_SDA_L();
    I2C_Delay_us(1);
    I2C_SCL_H();
    I2C_Delay_us(1);
    I2C_SDA_H();      // SDA拉高，产生停止沿
    I2C_Delay_us(1);
}

uint8_t Soft_I2C_Send_Byte(uint8_t dat)
{
    for(int8_t i=7; i>=0; i--)
    {
        I2C_SCL_L();
        if((dat & (0x01 << i)) != 0)
        {
            I2C_SDA_H();
        }
        else
        {
            I2C_SDA_L();
        }
        I2C_Delay_us(1);
        I2C_SCL_H();
        I2C_Delay_us(1);
    }
    // 读取ACK或者NAK
    I2C_SCL_L();
    I2C_SDA_H();
    I2C_Delay_us(1);
    I2C_SCL_H();
    I2C_Delay_us(1);
    uint8_t a=I2C_READ_SDA();
    return I2C_READ_SDA();    
}

uint8_t Soft_I2C_Read_Byte(uint8_t ack)
{
    uint8_t byte = 0;

    for(int8_t i=7; i>=0; i--)
    {
        I2C_SCL_L();
        I2C_SDA_H(); // 释放SDA总线
        I2C_Delay_us(1);
        I2C_SCL_H();
        I2C_Delay_us(1);

        if(I2C_READ_SDA()!= 0)
        {
            byte |= (0x01 << i);
        }
    }
    I2C_SCL_L();
    ack ? I2C_SDA_L() : I2C_SDA_H(); // 发送ACK或NACK
    I2C_Delay_us(1);
    I2C_SCL_H();
    I2C_Delay_us(1);
    return byte;
}

int8_t Soft_I2C_SendBytes(uint8_t Addr, uint8_t *pData, uint16_t Size)
{
    Soft_I2C_Start();

    if (Soft_I2C_Send_Byte(Addr & 0xfe) != 0)// 发送器件地址+写位,写位是0
    {
        Soft_I2C_Stop();
        return -1;
    }

    for (uint32_t i=0; i<Size; i++)
    {
        if (Soft_I2C_Send_Byte(pData[i]) != 0)
        {
            Soft_I2C_Stop();
            return -2;
        }
    }
    Soft_I2C_Stop();
    return 0;
}

int8_t Soft_I2C_RecvBytes(uint8_t Addr, uint8_t *pData, uint16_t Size)
{
    Soft_I2C_Start();

    if (Soft_I2C_Send_Byte(Addr | 0x01) != 0)// 发送器件地址+读位,读位是1
    {
        Soft_I2C_Stop();
        return -1;
    }

    for (uint32_t i=0; i<Size; i++)
    {
        pData[i] = Soft_I2C_Read_Byte(i < (Size - 1) ? 1 : 0);////
    }
    Soft_I2C_Stop();
    return 0;
}
