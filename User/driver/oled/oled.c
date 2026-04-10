#include "main.h"
#include "oled.h"
#include "i2c.h"
#include "soft_i2c.h"
#include "stm32f1xx_hal.h"
#include <string.h>
extern I2C_HandleTypeDef hi2c1;

uint8_t a[] = {0x00,0x8d,0x14,0xaf,0xa5};

uint8_t data[] = 
{    
    0xAE, // 1. 关闭显示 (0xAE，无参数) 
    0xD5, 0x80,
    // 2. 设置多路复用率 (0xA8 + 参数0x3F)
    0xA8, 0x3F,
    // 3. 设置显示偏移 (0xD3 + 参数0x00)
    0xD3, 0x00,
    // 4. 设置显示起始行 (0x40，无参数)
    0x40,
    0xA1,
    // 8. COM扫描方向（上下翻转）(0xC8，无参数)
    0xC8,
    // 9. COM引脚硬件配置 (0xDA + 参数0x12)
    0xDA, 0x12,
    // 10. 设置对比度 (0x81 + 参数0xCF)
    0x81, 0xCF,
    // 11. 设置预充电周期 (0xD9 + 参数0xF1)
    0xD9, 0xF1,
    // 12. 设置VCOMH电压倍率 (0xDB + 参数0x30)
    0xDB, 0x30,
    // 14. 正常显示模式（按显存显示）(0xA4，无参数)
    0xA4,
    // 15. 正显模式（非反显）(0xA6，无参数)
    0xA6,
    0X8D, 0X14, // 13. 电荷泵设置 (0x8D + 参数0x14)
    // 16. 开启OLED显示 (0xAF，无参数)
    0xAF
};

// 用于合并清屏的一页 128 字节缓冲区
static uint8_t zero_buf[128] = {0};
static void oled_set_pos(uint8_t x, uint8_t page);//设置坐标

static void oled_write_command(uint8_t cmd[], uint8_t lenth, uint16_t timeout)
{
    uint8_t addr = 0x78; // OLED I2C地址（0x3c）左移1位，末尾补0表示写操作
    uint8_t buffer[256]; // 缓冲区，假设最大长度足够
    buffer[0] = 0x00; // 命令模式控制字节
    if(lenth > 0)
    {
        memcpy(&buffer[1], cmd, lenth);
    }
    HAL_I2C_Master_Transmit(&hi2c1, addr, buffer, 1 + lenth, timeout); // 一次性发送控制字节和数据
}

static void oled_write_data(uint8_t data[], uint8_t lenth, uint16_t timeout)
{
    uint8_t addr = 0x78; // OLED I2C地址（0x78）左移1位，末尾补0表示写操作
    uint8_t buffer[256]; // 缓冲区，假设最大长度足够
    buffer[0] = 0x40; // 数据模式控制字节
    if(lenth > 0)
    {
        memcpy(&buffer[1], data, lenth);
    }
    HAL_I2C_Master_Transmit(&hi2c1, addr, buffer, 1 + lenth, timeout); // 一次性发送控制字节和数据
}

static void oled_clear(void)
{   
    for(uint8_t page = 0; page < 8; page++ )
    {
        oled_set_pos(0, page);
        oled_write_data(zero_buf, 128, 100);
    }
}

void oled_init(void)
{   
    HAL_Delay(50); // 延时等待设备稳定
    oled_write_command(data, sizeof(data) / sizeof(data[0]), 100);
    oled_clear();
 }

static void oled_set_pos(uint8_t x, uint8_t page)//设置坐标
{
    uint8_t cmd[3];
    cmd[0] = 0x00 | (x & 0x0F);           
    cmd[1] = 0x10 | ((x & 0xF0) >> 4);          
    cmd[2] = 0xB0 | page;   
    oled_write_command(cmd, 3, 100);
}

void oled_write_char(uint8_t x, uint8_t page, char ch, uint8_t size)//显示一个字符
{
    if(size == 8)
    {
    oled_set_pos(x, page);
    uint8_t c = ch - ' ';//得到偏移后的值
    oled_write_data((uint8_t *)&F6x8[c][0], 6, 100);
    }
    if(size == 16)
    {
        oled_set_pos(x, page);
        uint8_t c = ch - ' ';//得到偏移后的值
        oled_write_data((uint8_t *)&F8X16[c][0], 8, 100);
        oled_set_pos(x, page + 1);
        oled_write_data((uint8_t *)&F8X16[c][8], 8, 100);
    }
}

void oled_write_string(uint8_t x, uint8_t page, const char *s, uint8_t size)
{   
    if (size == 8)
    {
        uint8_t buf[128]; // 一页最多 128 列
        uint16_t idx = 0; // 缓冲区索引
        uint8_t cur_x = x;

        // 先设置起始位置（当前页）
        oled_set_pos(cur_x, page);

        while (*s != '\0')
        {
            uint8_t c = *s - ' ';

            // 如果当前缓冲不足以容纳下一个字符（6 字节），先发送已缓存的数据
            if (idx + 6 > sizeof(buf))
            {
                oled_write_data(buf, idx, 100);
                idx = 0;
                // 发送后继续在当前列 cur_x 写入下一段数据，需更新位置
                oled_set_pos(cur_x, page);
            }

            // 将字符点阵（6 字节）追加到缓冲
            for (uint8_t k = 0; k < 6; k++)
                buf[idx++] = F6x8[c][k];

            cur_x += 6; // 前进列坐标
            s++;
        }

        // 发送剩余数据
        if (idx > 0)
            oled_write_data(buf, idx, 100);

        return;
    }
    else if (size == 16)
    {
        while (*s != '\0')
        {
            oled_write_char(x, page, *s, size);
            x += 8;
            s++;
        }
    }
}
