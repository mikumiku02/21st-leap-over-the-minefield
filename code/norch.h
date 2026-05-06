
#ifndef _NORCH_H_
#define _NORCH_H_


typedef struct {
    float b0, b1, b2; // 分子系数
    float a1, a2;     // 分母系数
    float x1, x2;     // 历史输入数据 x[n-1], x[n-2]
    float y1, y2;     // 历史输出数据 y[n-1], y[n-2]
} NotchFilter;

extern NotchFilter x_acc_filter;
extern NotchFilter y_acc_filter;
extern NotchFilter z_acc_filter;
extern NotchFilter roll_gyro_filter;
extern NotchFilter pitch_gyro_filter;
extern NotchFilter yaw_gyro_filter;

void Update_Notch_Coeffs_all(void);
void Notch_Init_all(void);
void Notch_Init(NotchFilter*j);
void Update_Notch_Coeffs(NotchFilter *f, float f_noise, float fs);
float Notch_Update(NotchFilter* f, float input);
#endif

