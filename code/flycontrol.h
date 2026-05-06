/*********************************************************************************************************************
 * Name:flycontrol.h
 *  
 * Created on:
 * Author:Lsl
********************************************************************************************************************/

#ifndef _FLY_H_
#define _FLY_H_

extern PIDController hight;
extern PIDController Roll;
extern PIDController Roll_v;
extern PIDController Yaw;
extern PIDController Yaw_v;
extern PIDController Pitch;
extern PIDController Pitch_v;
extern PIDController XX;
extern PIDController YY;
extern PIDController PMW_X;
extern PIDController PMW_Y;
extern float hhhh;
extern float yyyy;
extern float yy_v;
extern float rrrr;
extern float rr_v;
extern float pppp;
extern float pp_v;
extern float offest_x;
extern float offest_y;
extern float pmw_xx;
extern float pmw_yy;
extern float target_xx;
extern float target_yy;
extern float highter;//¸ß¶È
extern int highter_1;
extern float PPXX;
extern float PPYY;

void PID_UPdate (void);
void pid_Init_all(float hight_p,float hight_i,float hight_d,float Roll_p,float Roll_i,float Roll_d,float Yaw_p,float Yaw_i,float Yaw_d,float Pitch_p,float Pitch_i,float Pitch_d,float XX_p,
                  float XX_i,float XX_d,float YY_p,float YY_i,float YY_d,float PMW_X_p,float PMW_X_i,float PMW_X_d,float PMW_Y_p,float PMW_Y_i,float PMW_Y_d);
void fly_star (void);
void PID_UPdate_1 (void);

#endif
