/*********************************************************************************************************************
 * Name:pid.h
 *  
 * Created on:
 * Author:
********************************************************************************************************************/

#ifndef _PID_H_
#define _PID_H_

typedef struct {
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float integral; // 积分项
    float derivative;
    float last_error; // 上一次误差
    float output;   // 输出值
    float error;
} PIDController;


// 函数声明
void  PID_Init(PIDController *pid, float kp, float ki, float kd);//初始化PID参数
float PID_Update(PIDController *pid, float setpoint, float measured_value,float max_integral);//PID更新

#endif /* CODE_ATTITUDE_CONTROL_H_ */
