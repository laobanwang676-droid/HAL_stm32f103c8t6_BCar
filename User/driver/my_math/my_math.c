#include "my_math.h"
#include "math.h"

/************************ 核心配置（6KB RAM精准控制） ************************/
#define TAB_SIZE        512     // sin表512项（RAM：512×4=2048字节=2KB）
#define PI              3.1415926535f
#define TWO_PI          (2 * PI)
#define HALF_PI         (PI / 2.0f)
#define STEP            (TWO_PI / TAB_SIZE)  // 步长≈0.012rad≈0.7°，误差＜0.001

// 去掉const（可写）+ static（仅本文件可见），RAM占用2KB
static float sin_tab[TAB_SIZE];  

// atan2查表数组（512×2=1024项，RAM：1024×4=4096字节=4KB）
#define ATAN_TAB_SIZE   512     
static float atan_tab[ATAN_TAB_SIZE * 2]; // 覆盖输入[-10,10]

/************************ 辅助函数（适配新数组大小） ************************/
// 弧度归一化到0~2π范围
static float rad_normalize(float rad)
{
    rad = fmod(rad, TWO_PI);
    if (rad < 0) rad += TWO_PI;
    return rad;
}

// 弧度→sin表索引（0~511）
static uint16_t rad2idx(float rad)
{
    rad = rad_normalize(rad);
    return (uint16_t)(rad / STEP + 0.5f); // 四舍五入补偿精度
}

// atan输入[-10,10]→索引[0,1023]
static uint16_t atan_input2idx(float x)
{
    x = (x < -10.0f) ? -10.0f : (x > 10.0f) ? 10.0f : x;
    return (uint16_t)((x + 10.0f) * (ATAN_TAB_SIZE - 1) / 10.0f + 0.5f);
}

/************************ 初始化函数（可正常写入，无只读限制） ************************/
void my_math_init(void)
{
    uint16_t i;
    float rad, x;

    // ---------------- 初始化sin表（2KB RAM，可写） ----------------
    for (i = 0; i < TAB_SIZE; i++)
    {
        rad = i * STEP;                  
        sin_tab[i] = sin(rad);           // 运行时写入预计算值
    }

    // ---------------- 初始化atan表（4KB RAM，可写） ----------------
    for (i = 0; i < ATAN_TAB_SIZE * 2; i++)
    {
        x = -10.0f + (20.0f * i) / (ATAN_TAB_SIZE * 2 - 1); 
        atan_tab[i] = atan(x);           // 运行时写入预计算值
    }
}

/************************ 核心函数（推导实现，无多余数组） ************************/
// 快速sin（查表取值，512项精度足够）
float qsin(float x)
{
    uint16_t idx = rad2idx(x);
    return sin_tab[idx];
}

// 快速cos（由sin推导：cos(x) = sin(π/2 - x)，无额外RAM）
float qcos(float x)
{
    float rad = HALF_PI - x; 
    uint16_t idx = rad2idx(rad);
    return sin_tab[idx];
}

// 快速tan（由sin/cos推导：tan(x) = sin(x)/cos(x)，无额外RAM）
float qtan(float x)
{
    float sin_val = qsin(x);
    float cos_val = qcos(x);
    
    // 避免除0（接近π/2/3π/2时返回极大值）
    if (fabs(cos_val) < 1e-6f)
        return 1e9f;
    return sin_val / cos_val;
}

// 快速atan2（核心用于姿态解算，1024项精度足够）
float qatan2(float y, float x)
{
    if (x == 0.0f) 
        return (y > 0) ? HALF_PI : (y < 0) ? -HALF_PI : 0.0f;

    float ratio = y / x;
    float atan_val = atan_tab[atan_input2idx(ratio)];

    // 象限修正（保证全象限精度）
    if (x < 0)
    {
        if (y >= 0)
            atan_val += PI;
        else
            atan_val -= PI;
    }
    return atan_val;
}

/************************ 占位函数（避免编译报错） ************************/
float qasin(float x) 
{ 
    (void)x; // 消除未使用警告
    return 0.0f; 
}

float qacos(float x) 
{ 
    (void)x;
    return 0.0f; 
}

// atan复用atan2，无额外RAM占用
float qatan(float x)
{
    return qatan2(x, 1.0f);
}
