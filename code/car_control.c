/*********************************************************************************************************************
 * Name:car_control.c
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "kaerman.h"
#include "quaternion.h"

#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
#define DIR_R1              (P09_1)
#define PWM_R1              (TCPWM_CH24_P09_0)
#define DIR_L1              (P05_1)
#define PWM_L1              (TCPWM_CH09_P05_0)

#define DIR_R2              (P10_3)
#define PWM_R2              (TCPWM_CH30_P10_2)
#define DIR_L2              (P05_3)
#define PWM_L2              (TCPWM_CH11_P05_2)

PIDController qianjin;
PIDController zuoyou;
PIDController suanzhuan;

void car_Init(float kp1,float ki1,float kd1,float kp2,float ki2, float kd2,float kp3,float ki3,float kd3){
    
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L1, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                           // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L2, 17000, 0);                                                 // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    PID_Init(&qianjin,kp1,ki1,kd1);
    PID_Init(&zuoyou,kp2,ki2,kd2);
    PID_Init(&suanzhuan,kp3,ki3,kd3);
}
void motor_control(int8 lq,int8 rq,int8 lh,int8 rh){
  if(lq >=0)
  {
     gpio_set_level(DIR_L1, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_L1, lq * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(lq < 0)
   {
     gpio_set_level(DIR_L1, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_L1, (-lq) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  
  ////////
  if(rq >=0)
  {
     gpio_set_level(DIR_R1, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_R1, rq * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(rq < 0)
   {
     gpio_set_level(DIR_R1, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_R1, (-rq) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  
  ////////
  if(lh >=0)
  {
     gpio_set_level(DIR_L2, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_L2, lh * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(lh < 0)
   {
     gpio_set_level(DIR_L2, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_L2, (-lh) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  
  ////////
  if(rh >=0)
  {
     gpio_set_level(DIR_R2, GPIO_HIGH);                                  // DIR输出高电平
     pwm_set_duty(PWM_R2, rh * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
  if(rh < 0)
   {
     gpio_set_level(DIR_R2, GPIO_LOW);                                  // DIR输出高电平
     pwm_set_duty(PWM_R2, (-rh) * (PWM_DUTY_MAX / 100));                  // 计算占空比
  }
}

void car_star(void){
  motor_control(10,-10,-10,10);
  
}

