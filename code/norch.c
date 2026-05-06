#include "zf_common_headfile.h"
#include "norch.h"

/**
 * @brief 陷波器核心计算 (直接型I结构)
 * @param f 陷波器结构体指针
 * @param input 传感器当前的原始值 (已转物理单位)
 * @return 滤波后的平滑值
 */
NotchFilter x_acc_filter;
NotchFilter y_acc_filter;
NotchFilter z_acc_filter;
NotchFilter roll_gyro_filter;
NotchFilter pitch_gyro_filter;
NotchFilter yaw_gyro_filter;

void Notch_Init(NotchFilter*j)
{
  j->b0 = 0;
  j->b1 = 0;
  j->b2 = 0;
  j->a1 = 0;
  j->a2 = 0;
  j->x1 = 0;
  j->x2 = 0;
  j->y1 = 0;
  j->y2 = 0; 
}

void Notch_Init_all(void)
{
  Notch_Init(&x_acc_filter);
  Notch_Init(&y_acc_filter);
  Notch_Init(&z_acc_filter);
  Notch_Init(&roll_gyro_filter);
  Notch_Init(&pitch_gyro_filter);
  Notch_Init(&yaw_gyro_filter);
  
  Update_Notch_Coeffs(&x_acc_filter,0.0,0.0);
  Update_Notch_Coeffs(&y_acc_filter,0.0,0.0);
  Update_Notch_Coeffs(&z_acc_filter,0.0,0.0);
  Update_Notch_Coeffs(&roll_gyro_filter,0.0,0.0);
  Update_Notch_Coeffs(&pitch_gyro_filter,0.0,0.0);
  Update_Notch_Coeffs(&yaw_gyro_filter,0.0,0.0);
}

void Update_Notch_Coeffs(NotchFilter *f, float f_noise, float fs) {
    // f_noise: 电脑分析出的噪声频率 (Hz)
    // fs: 采样频率 (500.0f)
    
    float r = 0.96f; // 带宽因子，越接近 1 坑越窄（延迟小，但频率精度要求高）
    float omega = 2.0f * 3.14159265f * f_noise / fs;
    float cos_w = cosf(omega);

    // 标准陷波器系数计算
    f->b0 = 1.0f;
    f->b1 = -2.0f * cos_w;
    f->b2 = 1.0f;
    f->a1 = -2.0f * r * cos_w;
    f->a2 = r * r;

    // 增益归一化：确保 0Hz（直流信号）增益为 1
    // 这样不会改变陀螺仪的灵敏度
    float scale = (1.0f + f->a1 + f->a2) / (f->b0 + f->b1 + f->b2);
    f->b0 *= scale;
    f->b1 *= scale;
    f->b2 *= scale;
}

void Update_Notch_Coeffs_all(void)
{
  Update_Notch_Coeffs(&x_acc_filter,0.0,0.0);
  Update_Notch_Coeffs(&y_acc_filter,0.0,0.0);
  Update_Notch_Coeffs(&z_acc_filter,0.0,0.0);
  Update_Notch_Coeffs(&roll_gyro_filter,0.0,0.0);
  Update_Notch_Coeffs(&pitch_gyro_filter,0.0,0.0);
  Update_Notch_Coeffs(&yaw_gyro_filter,0.0,0.0);
}

float Notch_Update(NotchFilter* f, float input) {
    // 计算差分方程: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) - a1*y(n-1) - a2*y(n-2)
    float output = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2 
                   - f->a1 * f->y1 - f->a2 * f->y2;

    // 更新历史状态 (滑动窗口)
    f->x2 = f->x1;    // x(n-2) = x(n-1)
    f->x1 = input;    // x(n-1) = x(n)
    
    f->y2 = f->y1;    // y(n-2) = y(n-1)
    f->y1 = output;   // y(n-1) = y(n)

    return output;
}


