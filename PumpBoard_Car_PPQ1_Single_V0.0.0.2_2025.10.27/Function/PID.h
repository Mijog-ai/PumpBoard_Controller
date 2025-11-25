/**********************************************************************
*
*   Copyright(C) 2021
*   FileName: PID.h
*   Version : V0.0.1
*   Date    : 2021-03-02
*   Author  : 
*   Description: PID Control Algorithm.
*   
*   Others:
*
*   Function List:
*   History      :
*
***********************************************************************/

#ifndef __PID_H__
#define __PID_H__


#include "stdint.h"
#include <stdio.h>
#include "stdlib.h"
#include <math.h>
#include <stdbool.h>
#include "limit.h"
#include "stm32f4xx.h"  



/**********************************************************************

*********电流环***********************************************/
#define CUR_A_KP                                    50
#define CUR_A_KI                                    6500//2048
#define CUR_A_KD                                    0//10
#define CUR_A_KP_DIV                                2048
#define CUR_A_KI_DIV                                8192
#define CUR_A_KD_DIV                                10
#define CUR_A_SUMMAX                                (CUR_A_OUTPUT_MAX * CUR_A_KI_DIV / CUR_A_KI)//(PWM_ARR * CUR_A_KI_DIV / CUR_A_KI)//(PWM_A_OUTPUT_MAX * PWM_A_KI_DIV / PWM_A_KI) 		//2600
#define CUR_A_SUMMIN                                0
#define CUR_A_OUTPUT_MAX                            (PWM_ARR * 8 / 10)      //PWM_ARR = 21000
#define CUR_A_OUTPUT_MIN                            0

/**********角度环***********************************************/
#define ANGLE_A_KP                                  150//1750//550//600//1788//50                  520
#define ANGLE_A_KI                                  0//20//1000        
#define ANGLE_A_KD                                  1500//280//100//10         35              10
#define ANGLE_A_KV                                  1000//300//75          //      100         30
#define ANGLE_A_KP_DIV                              400//10
#define ANGLE_A_KI_DIV                              32000//9090
#define ANGLE_A_KD_DIV                              450
#define ANGLE_A_KV_DIV                              350
#define ANGLE_A_SUMMAX                              (ANGLE_A_OUTPUT_MAX * ANGLE_A_KI_DIV / (ANGLE_A_KI + 1))		
#define ANGLE_A_SUMMIN                              (ANGLE_A_OUTPUT_MIN * ANGLE_A_KI_DIV / (ANGLE_A_KI + 1))
#define ANGLE_A_OUTPUT_MAX                          (2500)//(21922)//(18000)//(25000)//(PWM_ARR)//(PWM_ARR * 6 / 10)
#define ANGLE_A_OUTPUT_MIN                          (-2500)//(-21828)//0//-(ANGLE_A_OUTPUT_MAX)//-(PWM_ARR)//0
#define ANG_I_AREA                                   200
#define ANG_I_AREA_L                                 0
#define ANG_AREA_I                                   20
#define ANG_ERR_D_PERIOD                             1
/**********压力环***********************************************/
#define PRS_A_KP                                    1600//470//1788//50
#define PRS_A_KI                                    0//20//1000        
#define PRS_A_KD                                    515//80//100//10
#define PRS_A_KV                                    515
#define PRS_A_KP_DIV                                300//10
#define PRS_A_KI_DIV                                200//9090
#define PRS_A_KD_DIV                                100
#define PRS_A_KV_DIV                                100
#define PRS_A_SUMMAX                                (PRS_A_OUTPUT_MAX * PRS_A_KI_DIV / (PRS_A_KI + 1))		
#define PRS_A_SUMMIN                                (PRS_A_OUTPUT_MIN * PRS_A_KI_DIV / (PRS_A_KI + 1))//0
#define PRS_A_OUTPUT_MAX                            (5000)//(18000)//(25000)//(PWM_ARR)//(PWM_ARR * 6 / 10)
#define PRS_A_OUTPUT_MIN                            (0)//0//-(ANGLE_A_OUTPUT_MAX)//-(PWM_ARR)//0
#define PRS_I_AREA                                   100
#define PRS_I_AREA_L                                 0
#define PRS_AREA_I                                   10
#define PRS_ERR_D_PERIOD                             1
// 位置式PID计算-------------------------------------------------------------------------------*/
typedef struct
{
	u16 Kp;            // PID 比例(Kp)参数
	u16 Ki;            // PID 全局积分(Ki)参数
	u16 Kd;            // PID 微分(Kd)参数
    u16 Kv;
	float Kp_div;      // PID 比例(Kp)参数
	float Ki_div;      // PID 积分(Ki)参数
	float Kd_div;      // PID 微分(Kd)参数	
    u16 Kv_div;
    s16 g_s16_Kv_Err;
    s16 g_s16_AngInPrs_Kv_Err;
	s16 cur_error;     // 当前误差值
	s16 last_error;    // 上一次误差值
	s32 sum_error;     // 累加误差值error积分器
	s32 sum_max;       // 积分限幅上限
	s32 sum_min;       // 积分限幅下限
	s32 output_max;    // 输出限幅上限
	s32 output_min;    // 输出限幅下限
	s32 out_value;//u16 out_value;     // 输出结果
    u16 PI_area; //积分起作用的区间，上区间
    u16 PI_area_l;//积分起作用的区间，上区间
    u16 AREA_I;//积分起作用的区间的I
    u16 Err_D_Period; //Err周期
}pid_location_t;


static inline s16 clamp_s16(s32 v, s16 lo, s16 hi) {
    if (v < lo) return lo; if (v > hi) return hi; return (s16)v;
}
static inline s32 clamp_s32(s32 v, s32 lo, s32 hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
}



extern pid_location_t          Pwm_Output_A;
extern pid_location_t          Pwm_Output_B;
extern pid_location_t          Angle_Loop;
extern pid_location_t          Pressure_Loop;


u32 PID_Regulator(u16 set_value, u16 cur_value, pid_location_t *pid);
u16 Angle_PID_Regulator(u16 set_value, u16 cur_value, pid_location_t *pid);
u16 Pressure_PID_Regulator(u16 set_value, u16 cur_value, pid_location_t *pid);
void UpdateMapCmdData(pid_location_t *Ang, pid_location_t *Prs);


#endif // #ifndef __PID_H__

