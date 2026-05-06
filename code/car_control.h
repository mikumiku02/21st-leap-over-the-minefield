/*********************************************************************************************************************
 * Name:car_control.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#ifndef CODE_CAR_CONTROL_H_
#define CODE_CAR_CONTROL_H_

extern PIDController qianjin;
extern PIDController zuoyou;
extern PIDController suanzhuan;

void car_Init(float kp1,float ki1,float kd1,float kp2,float ki2, float kd2,float kp3,float ki3,float kd3)
void motor_control(int8 lq,int8 rq,int8 lh,int8 rh);
void car_star(void);

#endif