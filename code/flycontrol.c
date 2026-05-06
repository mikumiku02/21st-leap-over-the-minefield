/*********************************************************************************************************************
 * Name:flycontrol.c
 *      飞控
 * Created on:
 * Author:  田深深深深深深
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "matrix.h"
#include "kaerman.h"
#include "quaternion.h"
#include "norch.h"

#define MAX_DUTY                (30)                    // 最大 测试 占空比 30%
#define MIN_DUTY                (10)                    // 最小 测试 占空比 10%


#define PMW_CHANNEL_1           (TCPWM_CH19_P08_0)      // pwm输出通道1                                 
#define PMW_CHANNEL_2           (TCPWM_CH20_P08_1)      // pwm输出通道2                             
#define PMW_CHANNEL_3           (TCPWM_CH21_P08_2)      // pwm输出通道3                                                                   
#define PMW_CHANNEL_4           (TCPWM_CH22_P08_3)      // pwm输出通道4


PIDController hight;
PIDController Roll;
PIDController Roll_v;
PIDController Yaw;
PIDController Yaw_v;
PIDController Pitch;
PIDController Pitch_v;
PIDController XX;
PIDController YY;
PIDController PMW_X;
PIDController PMW_Y;
float hhhh;
float yyyy;
float yy_v;
float rrrr;
float rr_v;
float pppp;
float pp_v;
float offest_x;
float offest_y;
float pmw_xx;
float pmw_yy;
float target_xx;
float target_yy;
float highter;//高度
int highter_1;
float PPXX;
float PPYY;

void pid_Init_all(float hight_p,float hight_i,float hight_d,float Roll_p,float Roll_i,float Roll_d,float Yaw_p,float Yaw_i,float Yaw_d,float Pitch_p,float Pitch_i,float Pitch_d,float XX_p,
                  float XX_i,float XX_d,float YY_p,float YY_i,float YY_d,float PMW_X_p,float PMW_X_i,float PMW_X_d,float PMW_Y_p,float PMW_Y_i,float PMW_Y_d)
{
  PID_Init(&hight,hight_p,hight_i,hight_d);
  PID_Init(&Roll,Roll_p,Roll_i,Roll_d);
  PID_Init(&Yaw,Yaw_p,Yaw_i,Yaw_d);
  PID_Init(&Pitch,Pitch_p,Pitch_i,Pitch_d);


  PID_Init(&Roll_v,1.0,0.0,0.21);       //1.8,0.0,0.18
  PID_Init(&Pitch_v,1.0,0.0,0.16);     //1.4,0.0,0.1
  PID_Init(&Yaw_v,5.5,0.0,0.5);       //7.5，0.0，0.5


  PID_Init(&XX,XX_p,XX_i,XX_d);
  PID_Init(&YY,YY_p,YY_i,YY_d);
  PID_Init(&PMW_X,PMW_X_p,PMW_X_i,PMW_X_d);
  PID_Init(&PMW_Y,PMW_Y_p,PMW_Y_i,PMW_Y_d);

  pwm_init(PMW_CHANNEL_1, 400, 0);                    // 初始化pwm引脚功能  默认400hz  0%占空比
  pwm_init(PMW_CHANNEL_2, 400, 0);
  pwm_init(PMW_CHANNEL_3, 400, 0);
  pwm_init(PMW_CHANNEL_4, 400, 0);

}

void PID_UPdate_1 (void)
{
    
    pmw_xx=-PID_Update(&PMW_X,0+target_xx,PPXX+0.25f,100);//光流
    pmw_yy=-PID_Update(&PMW_Y,0+target_yy,PPYY-0.65f,100);
  
}


void PID_UPdate (void)
{
   /* target_xx = PID_Update(&XX,94.0,offest_x,100);//摄像头
    target_yy = PID_Update(&YY,60.0,offest_y,100);*/
    if(highter<1500)
    {
      highter_1=highter;
      hhhh= PID_Update(&hight,900.0,highter_1,100.0);}//TOF 
    else
    {
    highter=0;
    }
    /*pmw_xx=PID_Update(&PMW_X,0+target_xx,PPXX,100);//光流
    pmw_yy=PID_Update(&PMW_Y,0+target_yy,PPYY,100);*/

    yyyy= -PID_Update(&Yaw,0,s_angle.agz,100);//电机要的 s_angle.agz
    rrrr= -PID_Update(&Roll,0.0+pmw_xx,s_angle.agx+3.4f,100); //s_angle.agx-2.8
    pppp= -PID_Update(&Pitch,0.0+pmw_yy,s_angle.agy+1.08f,100);//s_angle.agy+1.5
    //printf("%f,%f,%f\n",s_angle.agx,s_angle.agy,s_angle.agz);
    //printf("%d\n",dl1b_distance_mm);
  
}

void fly_star (void)
{   

    float rv = PID_Update(&Roll_v,rrrr,imu660rb_gyro_x_l,100);//imu660rc_gyro_x_2
    float pv = PID_Update(&Pitch_v,pppp,imu660rb_gyro_y_l,100);
    float yv = PID_Update(&Yaw_v,yyyy,imu660rb_gyro_z_l,100);

    int32 yyy = (float)-yv;

    int32 hhh=0;// (float)hhhh;

    int32 rrr = (float)-rv;

    int32 ppp = (float)pv;
   // printf("%d,\n",yyy);

    uint32 M1=6175+hhh/4+rrr/4+ppp/4 -yyy/4;
    uint32 M2=6175+hhh/4+rrr/4-ppp/4 +yyy/4;
    uint32 M3=6175+hhh/4-rrr/4+ppp/4 +yyy/4;
    uint32 M4=6175+hhh/4-rrr/4-ppp/4 -yyy/4;
  // printf("hhh:%d\n",hhh);
   // printf("ppp:%d\n",ppp);
   // printf("rrr:%d\n",rrr);
    if(M1>7500) M1=7500;
    if(M2>7500) M2=7500;
    if(M3>7500) M3=7500;
    if(M4>7500) M4=7500;

   printf("%d,%d,%d,%d\n",M1,M2,M3,M4);

    pwm_set_duty(PMW_CHANNEL_1,M1 );//+rrr +ppp -yyy                      // 输出占空比数值  输出值 = 占空比设置值 + 1ms固定占空比
    pwm_set_duty(PMW_CHANNEL_2,M2);//+rrr -ppp +yyy
    pwm_set_duty(PMW_CHANNEL_3,M3);//-rrr +ppp +yyy
    pwm_set_duty(PMW_CHANNEL_4,M4);//-rrr -ppp-yyy
}
