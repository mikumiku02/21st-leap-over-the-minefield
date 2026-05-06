/*********************************************************************************************************************
 * Name:quaternion.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#ifndef _QUATERNION_H_
#define _QUATERNION_H_
typedef struct
{
    float kp;
    float ki;
    float qdt;
}my_qu;
typedef struct
{
    float agx;
    float agy;
    float agz;
}quat_angle;
typedef struct
{
   float ac_x;
   float ac_y;
   float ac_z;
   float gy_x;
   float gy_y;
   float gy_z;
}chw_angle;

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
}siyuanshu;
extern my_qu sc;
extern quat_angle s_angle;
extern siyuanshu s_shu;
extern chw_angle cl_angle;
extern float imu660rb_acc_x_1;
extern float imu660rb_acc_y_1;
extern float imu660rb_acc_z_1;
extern float imu660rb_acc_x_0;
extern float imu660rb_acc_y_0;
extern float imu660rb_acc_z_0;
    
void quaternion_Init(my_qu*liu,quat_angle*aksq,siyuanshu*siyaun ,float kkp,float kki,float pit_time);
void my_quaternion(my_qu*liu,quat_angle*aksq,siyuanshu*siyaun);
void imu_calibration(void);
//void dtlb(void);
void ycl(void);
#endif
