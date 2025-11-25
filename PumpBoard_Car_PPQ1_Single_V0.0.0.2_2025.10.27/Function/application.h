#ifndef __APPLICATION_H
#define __APPLICATION_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "HengLiPumpCmd.h"
#include "string.h"
#include "flash.h"
#include "PID.h"

#define SW_VERS                                     (s32)(0x0105)

#define PPQ_1                                                        //PPQ_1: Dual solenoid valves

#define WAY_3
#define old_way



//PPRE2: SYS_CLK divided by 2, frequency is 84Mhz
//PPRE1: SYS_CLK divided by 4, frequency is 42Mhz, TIM2 clock, because PPRE1 is SYS_CLK divided by 4, TIM clock frequency needs to be multiplied by 2, becomes 84MHZ         2024.05.19    Zp
#define MODULE_FREQ                                 (200)                                          //1K modulation frequency
#define ANGSHAKEFREQ                                (100)//(100)//(60)                                     //Dither frequency
#define ANGSHAKEAMP                                 (0.02f)
#define SYS_CLK                                     (42000000)
#define TIM2_PSC                                    (3)
#define CLKIM                                       (SYS_CLK / (TIM2_PSC + 1))                      //21MHZ
#define PWM_ARR                                     (CLKIM / MODULE_FREQ)                           //PWM period value     21000

/*ADI7949 AD precision is 14 bits*/
//Angle
#define ANG_FDB_MIN_ADC_VAL                         8902//9382//3040//3026//1930//1600//380      //Minimum feedback angle ADC
#define ANG_FDB_MID_ADC_VAL							9546//8400																		//Feedback angle ADC middle value
#define ANG_FDB_MID_ADC_VAL_SCOPE                   514//1013    //Feedback angle ADC middle value corresponding range
#define ANG_FDB_MAX_ADC_VAL                         15155//29400//15287//10455//6400//1590                                          //Maximum feedback angle ADC
#define ANG_PERUNIT_SCOPE                           5000//10000//12000                                                             //Per unit range
//Pressure     Due to unknown sensor reason, there is a problem with feedback pressure calibration, needs recalibration    ZP_2024.07.01
#define PRS_FDB_MIN_ADC_VAL                         2000
#define PRS_FDB_MID_ADC_VAL  						300
#define PRS_FDB_MID_ADC_VAL_SCOPE					10		//Unit: bar  
#define PRS_FDB_MAX_ADC_VAL                         34344	//2000  //1800--21Mpa   2000--25Mpa     2200--29
#define PRS_FDB_MAX_ADC_VAL_SCOPE					600		//Unit: bar

//#define PRS_MAX_VAL                               347   //Limited pressure value 200Bar
//Displacement
#define PUMP_MIN_CC_VAL                             0
#define PUMP_MAX_CC_VAL                             28
//Limited torque value--Displacement
#define PUMP_MIN_TOR_VAL                            0
#define PUMP_MAX_TOR_VAL                            250

#define VALVE_4MA_VAL                               0x3264                 //0x1500: 5376
#define VALVE_MID_POSI_VAL                          0x87A8                 //0x2AD8: 10968
#define VALVE_20MA_VAL                              0xDD4A                 //0x3FFF: 16383
/*=======================Time constants========================================================================================================*/
#define ANG_ERR_D_CYCLE                             2                      //         Angle error differential period
#define PRS_ERR_D_CYCLE                             2                      //         
//First-order filter time constant--a
#define ANG_REF_FILTER_TIM_CONST                    (0.0065f)//(0.0008f)
#define ANG_FDB_FILTER_TIM_CONST                    (0.015f)//(0.0089f)
#define PRS_FDB_FILTER_TIM_CONST                    (0.059f)
#define	PWR_FDB_FILTER_TIM_CONST					(0.001f)
//Differential period
#define ANG_D_FILTER_CNT                            50//160//200//60//50                     //50 corresponds to 5ms      Should be consistent with test and dither frequency      
#define PRS_D_FILTER_CNT                            50                     //         
//PID loop---Control period 100us
#define ANG_PI_LOOP_CYCLE                           10//50
#define PRS_PI_LOOP_CYCLE                           25
//Current loop period
#define CUR_PI_LOOP_CYCLE                           1

#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

/*=======================Conversion coefficients========================================================================================================*/
#define PWRCHGTORRATIO                              (58.8475f)              //Torque conversion coefficient, used for power conversion


#define HYSTERESIS_BAND         100     // Hysteresis band width, unit is Q15 format
#define MAX_CURRENT_RATE        500     // Maximum current change rate (mA/cycle)


// Control parameters
#define PRS_SAFETY_MARGIN          200U      // Safety margin
#define PRS_APPROACH_OFFSET        300U      // Approach pre-offset
#define PRS_FAST_APPROACH_THRESH   1000U     // Fast approach threshold
#define PRS_SLOW_APPROACH_THRESH   500U    	 // Slow approach threshold
#define RAMP_RATE_FAST             50U       // Fast ramp rate
#define RAMP_RATE_NORMAL           20U       // Normal ramp rate
#define RAMP_RATE_SLOW             5U        // Slow ramp rate
#define PRESSURE_CHANGE_RATE_MAX   100U      // Maximum pressure change limit value

typedef struct
{
    u16 u16AngAdc_MIN;
	u16	u16AngAdc_MID;
	u16	u16AngAdc_MID_Scope;
    u16 u16AngAdc_MAX;
    u16 u16AngPerUnitScope;
    u16 u16PrsAdc_MIN;
	u16 u16PrsAdc_MID;
	u16	u16PrsAdc_MID_Scope;
    u16 u16PrsAdc_MAX;
    u16	u16PrsAdc_MAX_Scope;
	
    u16 u16PumpCC_MIN;
    u16 u16PumpCC_MAX;
//    u16 u16TorLimit_MIN;
//    u16 u16TorLimit_MAX;
    u16 u16Valve_4mA_Dac;    
    u16 u16Valve_20mA_Dac;
    u16 u16Valve_MidPosi_Dac;
}_SYS_PARA;

#define _SYS_PARA_DEFAULT   {ANG_FDB_MIN_ADC_VAL, ANG_FDB_MID_ADC_VAL, ANG_FDB_MID_ADC_VAL_SCOPE, ANG_FDB_MAX_ADC_VAL, ANG_PERUNIT_SCOPE, PRS_FDB_MIN_ADC_VAL, \
							 PRS_FDB_MID_ADC_VAL, PRS_FDB_MID_ADC_VAL_SCOPE, PRS_FDB_MAX_ADC_VAL, PRS_FDB_MAX_ADC_VAL_SCOPE, PUMP_MIN_CC_VAL, PUMP_MAX_CC_VAL, \
							 VALVE_4MA_VAL, VALVE_20MA_VAL, VALVE_MID_POSI_VAL}

typedef struct
{
    u16  g_u16_Angle_Err_D_Period;
    u16  g_u16_Prs_Err_D_Period;
    u16  g_u16_Ang_Fdb_D_Period;            //Angle feedback differential period
    u16  g_u16_Prs_Fdb_D_Period;
    u16  g_u16_Ang_PI_LOOP_Period;
    u16  g_u16_Prs_PI_LOOP_Period;
	u16  g_u16_Cur_PI_LOOP_Period;
}_TIM_PARA;

#define _TIM_PARA_DEFAULT   {ANG_ERR_D_CYCLE,PRS_ERR_D_CYCLE,ANG_D_FILTER_CNT,PRS_D_FILTER_CNT,ANG_PI_LOOP_CYCLE,PRS_PI_LOOP_CYCLE,CUR_PI_LOOP_CYCLE}

typedef struct
{
    u16  g_u16_Ang_Fdb_D_Cnt;            //Angle feedback differential period
    u16  g_u16_Prs_Fdb_D_Cnt;
    u16  g_u16_Ang_PI_LOOP_Cnt;
    u16  g_u16_Prs_PI_LOOP_Cnt;
	u16  g_u16_PARA_SAVE_Cnt;
	u16  g_u16_Cur_PI_LOOP_Cnt;
}_TIM_CNT;

typedef struct
{
    float   g_f32_Ang_Ref_FilterTimPara;
    float   g_f32_Ang_Fdb_FilterTimPara;
    float   g_f32_Prs_Fdb_FilterTimPara;
	float   g_f32_Pwr_Ref_FilterTimPara;
}_LOW_PASS_FILTER_PARA;

#define _LOW_PASS_FILTER_PARA_DEFAULT   {ANG_REF_FILTER_TIM_CONST,ANG_FDB_FILTER_TIM_CONST,PRS_FDB_FILTER_TIM_CONST,PWR_FDB_FILTER_TIM_CONST}

typedef struct
{
    s32 g_s32_CurFdb_A;                         //mA
    s32 g_s32_CurFdb_B;
    u32 g_u32_AngFdb;                           //Per unit value
    float g_f32_PressureFdb;
    u32 g_u32_PressureFdb;                      //Per unit value
    
//    float g_f32_PressureFdb_D;
    s32 g_s32_PressureFdb_D;                    //Pressure feedback differential correspondence %   
    s32 g_s32_AngFdb_D;
}_FEEDBACK_VALUE;

#define _FEEDBACK_VALUE_DEFAULT         {0,0,0,0,0,\
                                         0,0}

typedef struct
{
    u16 g_u16_AngSwinging_Freq;
    float g_f32_AngSwinging_Amp;
}_SWING_PARA;
    
#define _SWING_PARA_DEFAULT         {ANGSHAKEFREQ, ANGSHAKEAMP}

typedef struct
{
    u8  PumpStrt_Ena;
    u8  AngLeak_Ena;
    u8  PrsLoop_Ena;
    u8  PwrLoop_Ena;
    u8  ParaSaveEna;                //Parameter save enable        2024.08.01    Zp
}_PPQ_ENA_CMD;

#define _PPQ_ENA_CMD_DEFAULT        {0, 0, 0, 0, 0}

typedef struct
{
    u32 g_u32_Test_PWMVal_A;
    u32 g_u32_Test_PWMVal_B;    
    u32 Cur_A_Ref;
    u32 Cur_B_Ref;
    
    u16 TiltAngRef;
    u16 g_u16_PerUnitPrsVal;
    float g_f32_TorqueLimt;
    
}_CMD_GEN;

#define _CMD_GEN_DEFAULT    {0,0,0,0,0,10000,PUMP_MAX_TOR_VAL}
                           


typedef struct
{
	_SYS_PARA               *SysPara;
    _TIM_PARA               *TimPeriod;
    _LOW_PASS_FILTER_PARA   *LowPassFilterTimConst;
    _SWING_PARA             *AngSwingPara;
    _PPQ_ENA_CMD            *PpqEna;
    float                   g_f32_PrsCompAngRatio;        //Pressure compensation coefficient, swash plate angle regression industrial process, can adjust pump displacement. Test data        2024.06.18    Zp
    u16                     g_u16_AngleLoop_P_Offset;
    float                   g_f32_leakage_compensation;   //Leakage coefficient        2024.06.18    Zp
    float                   g_f32TorqueRatio;
//    float       g_f32_Ang_Ref_FilterPara;
}_Parameter_Setting;

#define _PARA_SET_DEFAULT               {&SysParameter, &TimParametr, &FilterTimConstant, &AngSwinging, &PpqEnable, 0, 100, 0.0005f, PWRCHGTORRATIO}


typedef struct
{
    s16 g_s16_Ang_OffsetCur;
    s16 g_s16_Prs_OffsetCur;
    s16 g_s16_OffsetCur;
    u16 g_u16_AngRef;
    float g_f32_Filter_SwivelAngle;
    float g_f32_Filter_PressureData;
}_SCOPE;



extern  float               g_f32_CurA_Ratio;
extern  float               g_f32_CurB_Ratio;
extern  s32                 g_HLCmdMap[HL_CMDMAP_LEN];
extern  u16                 g_PCSetCmd[HL_WRCMD_LEN];
extern  u32                 g_PCRdCmd[HL_RDCMD_LEN];
extern  _Parameter_Setting  User_Parameter;
extern  _SWING_PARA         AngSwinging;
extern  _TIM_CNT            TimCnt;
extern  _PPQ_ENA_CMD        PpqEnable;
extern  _SYS_PARA           SysParameter;
extern  _SCOPE              ScopeVal;
extern  _TIM_PARA           TimParametr;
extern  _LOW_PASS_FILTER_PARA   FilterTimConstant;
extern  _FEEDBACK_VALUE     DetectorFdbVal;
extern  _CMD_GEN            InstructionSet;

extern  u32                 g_u32_PWMOutput_A;
extern  u32                 g_u32_PWMOutput_B;
extern  u8                  g_u8_ExchangeFlg;

void SensorModeChangeFunc(void);
void CurDetectionRangeChose(void);
void LED_Blink_Func(void);
void Cal_SolenoidCur(void);
void Cal_AngFbkVal_Func(float *p);
void Cal_PrsFbkVal_Func(float *p);
void Cal_PrsFbkDiff_Func(void);
void pwm_output_A_B_process(void);
void Solenoid_A_Set_PWM(u32);
void Solenoid_B_Set_PWM(u32);
void TiltAng_A_FreqShakeFunc(u32 *p);
void TiltAng_B_FreqShakeFunc(u32 *p);
u16 FeedbackSignalPerUnitFunc(u16,u16,u16,u16);
u16 FeedbackSignalPerUnitFunc_AngMID(u16,u16,u16,u16);
void TiltAngleLoop_Func(void);
void TimPeriodCnt_Func(void);
void CalAD7689AvgAngPrsData(void);
void Cur_A_B_RefFilterFunc(s16);

u8 LimitAngLoopOutput(u16,u16 *p,u16,u16,u16);
void update_control(void);
void Update_PWM_PidPara(pid_location_t *Cur_A, pid_location_t *Cur_B, pid_location_t *Ang, pid_location_t *Prs);
#endif
