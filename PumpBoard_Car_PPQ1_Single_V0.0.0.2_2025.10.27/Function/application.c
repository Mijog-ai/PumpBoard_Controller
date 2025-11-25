#include "application.h"  
#include "CheckTable.h"
#include "limit.h"
#include "AD7689.h"
#include "Hw_ADC.h"
#include "gpio.h"
#include "stdbool.h"
#include "deadzone_current.h"
/*=======================================================================================================
void SensorModeChangeFunc();
void CurDetectionRangeChose();
void Cal_SolenoidCur();
void Cal_AngFbkVal_Func();
void Cal_PrsFbkVal_Func();
void Cal_TorqueVal_Func();
u16 FeedbackSignalPerUnitFunc(u16 input_value,u16 Scope,u16 signal_min,u16 signal_max);
void pwm_output_A_B_process();
void Solenoid_A_Set_PWM(u32 SetPWMVal);
void Solenoid_B_Set_PWM(u32 SetPWMVal);
void Update_PWM_PidPara(pid_location_t *cur,pid_location_t *Ang);
void TiltAng_A_FreqShakeFunc(u16 *p);
void TiltAng_B_FreqShakeFunc(u16 *p);
void TiltAngleLoop_Func();
void LimitAngLoopOutput(u16 TiltAngRef,u16 *CurRef,u16 MinCurRef,u16 MaxCurRef);
=========================================================================================================*/
float                   g_f32_CurA_Ratio = 0;                                       // Current conversion coefficient
float                   g_f32_CurB_Ratio = 0;

u32                     g_u32_PWMOutput_A = 0;
u32                     g_u32_PWMOutput_B = 0;

u32											temple_set_Cur_B_Ref = 0;
u16											temple_offset_0_cur = 200;

_CMD_GEN                InstructionSet = _CMD_GEN_DEFAULT;
_Parameter_Setting      User_Parameter = _PARA_SET_DEFAULT;
_SWING_PARA             AngSwinging = _SWING_PARA_DEFAULT;
_PPQ_ENA_CMD            PpqEnable   = _PPQ_ENA_CMD_DEFAULT;
_SYS_PARA               SysParameter = _SYS_PARA_DEFAULT;
_TIM_CNT                TimCnt;
_SCOPE                  ScopeVal;





// Define a global variable
#ifdef WAY_1
static int16_t last_Cur_B_Ref = 0;
static int16_t last_offset = 0;
static u16 last_angle = 0;
#endif





/*******************************************************************************
* Function Name : SensorModeChangeFunc
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
#define  ANALOG_OUTPUT_2_CUR_MODE       (0)
#define  ANALOG_OUTPUT_1_CUR_MODE       (0)     
#define  ANGLE_SENSOR_CUR_MODE          (0)  		// 0: Voltage, 1: Current
#define  PRESSURE_SENSOR_CUR_MODE       (0)
    
#define  ANGLE_01_SENSOR_CUR_GAIN          (1)  		// 0: 0.4x gain, 1: 0.3x gain
#define  ANGLE_02_SENSOR_CUR_GAIN          (1)
#define  PRESSURE_01_SENSOR_CUR_GAIN       (1)
#define  PRESSURE_02_SENSOR_CUR_GAIN       (1)

void SensorModeChangeFunc()
{
	// Mode selection
    #if (ANALOG_OUTPUT_2_CUR_MODE == 0x01)
        GPIO_SetBits(ANALOG_OUTPUT_2_MODE_GPIO,ANALOG_OUTPUT_2_MODE_PIN);
    #else
        GPIO_ResetBits(ANALOG_OUTPUT_2_MODE_GPIO,ANALOG_OUTPUT_2_MODE_PIN);
    #endif
    
    #if (ANALOG_OUTPUT_1_CUR_MODE == 0x01)
        GPIO_SetBits(ANALOG_OUTPUT_1_MODE_GPIO,ANALOG_OUTPUT_1_MODE_PIN);
    #else
        GPIO_ResetBits(ANALOG_OUTPUT_1_MODE_GPIO,ANALOG_OUTPUT_1_MODE_PIN);
    #endif
    
    #if (ANGLE_SENSOR_CUR_MODE == 0x01)
        GPIO_SetBits(ANGLE_01_SENSOR_MODE_GPIO,ANGLE_SENSOR_MODE_PIN);
		GPIO_SetBits(ANGLE_02_SENSOR_MODE_GPIO,ANGLE_SENSOR_MODE_PIN);
    #else
        GPIO_ResetBits(ANGLE_01_SENSOR_MODE_GPIO,ANGLE_01_SENSOR_MODE_PIN);
		GPIO_ResetBits(ANGLE_02_SENSOR_MODE_GPIO,ANGLE_02_SENSOR_MODE_PIN);
    #endif
    
    #if (PRESSURE_SENSOR_CUR_MODE == 0x01)
        GPIO_SetBits(PT_01_SENSOR_MODE_GPIO,PT_01_SENSOR_MODE_PIN);
        GPIO_SetBits(PT_02_SENSOR_MODE_GPIO,PT_02_SENSOR_MODE_PIN);
    #else
        GPIO_ResetBits(PT_01_SENSOR_MODE_GPIO,PT_01_SENSOR_MODE_PIN);
        GPIO_ResetBits(PT_02_SENSOR_MODE_GPIO,PT_02_SENSOR_MODE_PIN);
    #endif

	// Gain selection
	#if (ANGLE_01_SENSOR_CUR_GAIN == 0x01)
        GPIO_SetBits(ANGLE_01_SENSOR_GAIN_GPIO,ANGLE_01_SENSOR_GAIN_PIN);
    #else
        GPIO_ResetBits(ANGLE_01_SENSOR_GAIN_GPIO,ANGLE_01_SENSOR_GAIN_PIN);
    #endif
    
    #if (ANGLE_02_SENSOR_CUR_GAIN == 0x01)
        GPIO_SetBits(ANGLE_02_SENSOR_GAIN_GPIO,ANGLE_02_SENSOR_GAIN_PIN);
    #else
        GPIO_ResetBits(ANGLE_02_SENSOR_GAIN_GPIO,ANGLE_02_SENSOR_GAIN_PIN);
    #endif
    
    #if (PRESSURE_01_SENSOR_CUR_GAIN == 0x01)
        GPIO_SetBits(PT_01_SENSOR_GAIN_GPIO,PT_01_SENSOR_GAIN_PIN);
    #else
        GPIO_ResetBits(PT_01_SENSOR_GAIN_GPIO,PT_01_SENSOR_GAIN_PIN);
    #endif
    
    #if (PRESSURE_02_SENSOR_CUR_GAIN == 0x01)
        GPIO_SetBits(PT_02_SENSOR_GAIN_GPIO,PT_02_SENSOR_GAIN_PIN);
    #else
        GPIO_ResetBits(PT_02_SENSOR_GAIN_GPIO,PT_02_SENSOR_GAIN_PIN);
    #endif	
	
}
/*******************************************************************************
* Function Name : CurDetectionRangeChose
* Description   : Determine range and gain    Small range [0, 2A]     Large range [0, 3.2A]
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void CurDetectionRangeChose()
{
    u8 t_u8_Scope_A_Chose = 1;          // 1 --> Large range     0 --> Small range
    u8 t_u8_Scope_B_Chose = 1;
	  // Solenoid A
    if (t_u8_Scope_A_Chose == 1)       // Large range
    {
        GPIO_SetBits(CUR_A_GAIN_GPIO,CUR_A_GAIN_PIN);
    }
    else if (t_u8_Scope_A_Chose == 0)
    {
        GPIO_ResetBits(CUR_A_GAIN_GPIO,CUR_A_GAIN_PIN);
    }
    else{}
    // Solenoid B
    if (t_u8_Scope_B_Chose == 1)
    {
        GPIO_SetBits(CUR_B_GAIN_GPIO,CUR_B_GAIN_PIN);
    }
    else if (t_u8_Scope_B_Chose == 0)
    {
        GPIO_ResetBits(CUR_B_GAIN_GPIO,CUR_B_GAIN_PIN);
    }
    else{}

    // Amplification factor selection_A
    if (t_u8_Scope_A_Chose == 0)
        g_f32_CurA_Ratio = 0.038019f;// Range not used
    else
        g_f32_CurA_Ratio = 0.071809f;
    // Amplification factor selection_B
    if (t_u8_Scope_B_Chose == 0)
        g_f32_CurB_Ratio = 0.038019f;// Range not used
    else
        g_f32_CurB_Ratio = 0.071809f;
}

/*******************************************************************************
* Function Name : Cal_SolenoidCur
* Description   : Calculate and obtain solenoid valve current
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Cal_SolenoidCur()
{
//    if (g_u8_Test_Flg == 0)
    DetectorFdbVal.g_s32_CurFdb_A = (s32)(g_f32_CurA_Ratio * AnalogInput.g_u32_CURRENT_A);
//    else
//        DetectorFdbVal.g_s32_CurFdb_A = 380;//AnalogInput.g_u16_CURRENT_A;
    
    DetectorFdbVal.g_s32_CurFdb_B = (s32)(g_f32_CurB_Ratio * AnalogInput.g_u32_CURRENT_B);
}
/*******************************************************************************
* Function Name : TimPeriodCnt_Func
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void TimPeriodCnt_Func()
{
    /* Angle differential filter */
    if (TimCnt.g_u16_Ang_Fdb_D_Cnt ++ >= User_Parameter.TimPeriod->g_u16_Ang_Fdb_D_Period)
        TimCnt.g_u16_Ang_Fdb_D_Cnt = 0;
     /* Pressure differential filter */
    if (TimCnt.g_u16_Prs_Fdb_D_Cnt ++ >= User_Parameter.TimPeriod->g_u16_Prs_Fdb_D_Period)
        TimCnt.g_u16_Prs_Fdb_D_Cnt = 0;
     /* Angle feedback period */
    if (TimCnt.g_u16_Ang_PI_LOOP_Cnt ++ >= User_Parameter.TimPeriod->g_u16_Ang_PI_LOOP_Period)
        TimCnt.g_u16_Ang_PI_LOOP_Cnt = 0;
    /* Pressure loop calculation */
    if (TimCnt.g_u16_Prs_PI_LOOP_Cnt ++ >= User_Parameter.TimPeriod->g_u16_Prs_PI_LOOP_Period)
        TimCnt.g_u16_Prs_PI_LOOP_Cnt = 0;
	/* Current loop calculation */
    if (TimCnt.g_u16_Cur_PI_LOOP_Cnt ++ >= User_Parameter.TimPeriod->g_u16_Cur_PI_LOOP_Period)
        TimCnt.g_u16_Cur_PI_LOOP_Cnt = 0;
}

/*******************************************************************************
* Function Name :
* Description   : Sensor signal per unit
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u16 FeedbackSignalPerUnitFunc(
    u16 input_value,
    u16 sensor_mid,
    u16 sensor_max,
    u16 output_mid      // Output value corresponding to midpoint
)
{
    if(sensor_max <= sensor_mid) return 0; // Ensure max > midpoint

    s32 raw = (s32)input_value;
    const s32 output_max = 5000;         // Fixed maximum output value

    float slope = (float)(output_max - output_mid) / (sensor_max - sensor_mid);

    // Perform linear mapping calculation
    s32 output = output_mid + (raw - sensor_mid) * slope;

    output = (output > output_max) ? output_max : (output < 0) ? 0 : output;    // Limit output to 0
            
    return (u16)output;
}

/*******************************************************************************
* Function Name :
* Description   : Sensor signal per unit, midpoint fixed
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/

u16 FeedbackSignalPerUnitFunc_AngMID(
    u16 input_value,
    u16 sensor_mid,
    u16 sensor_max,
    u16 output_mid      // Output value corresponding to midpoint
)
{
    if(sensor_max <= sensor_mid) return 0; // Ensure max > midpoint

    s32 raw = (s32)input_value;
    const s32 output_max = 5000;//10000;         // Fixed maximum output value

    float slope = (float)(output_max - output_mid) / (sensor_max - sensor_mid);

    // Perform linear mapping calculation
    s32 output = output_mid + (raw - sensor_mid) * slope;

    output = (output > output_max) ? output_max : (output < 0) ? 0 : output;    // Limit output to 0
            
    return (u16)output;
}
/*******************************************************************************
* Function Name : pwm_output_A_B_process
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
#define VOLT_OPN_LOOP               (0)
void pwm_output_A_B_process()
{
    //u16 t_u16_PWMOutput_A = 0;
    u32 t_u32_PWMOutput_B = 0;
   

    t_u32_PWMOutput_B = PID_Regulator(InstructionSet.Cur_B_Ref, (u16)DetectorFdbVal.g_s32_CurFdb_B, &Pwm_Output_B); 
    g_u32_PWMOutput_B = t_u32_PWMOutput_B; 
   
    
    Solenoid_B_Set_PWM(t_u32_PWMOutput_B);	

}
/*******************************************************************************
* Function Name : Solenoid_A_Set_PWM
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Solenoid_A_Set_PWM(u32 SetPWMVal)
{
    TIM_SetCompare3(TIM2, SetPWMVal); 
}
/*******************************************************************************
* Function Name : Solenoid_B_Set_PWM
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Solenoid_B_Set_PWM(u32 SetPWMVal)
{
    TIM_SetCompare4(TIM2, SetPWMVal); 
}
/*******************************************************************************
* Function Name : Update_PWM_PidPara
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Update_PWM_PidPara(pid_location_t *Cur_A, pid_location_t *Cur_B, pid_location_t *Ang, pid_location_t *Prs)
{
    if (g_HLCmdMap[0] == 0x5A5A5A5A)
    {        
        Cur_A->Kp = CUR_A_KP;
        Cur_A->Ki = CUR_A_KI;
        Cur_A->Kd = CUR_A_KD;
        Cur_A->Kp_div = CUR_A_KP_DIV;
        Cur_A->Ki_div = CUR_A_KI_DIV; 
        Cur_A->Kd_div = CUR_A_KD_DIV;
        Cur_A->Kv_div = 1024;
        Cur_A->sum_max = CUR_A_SUMMAX;
        Cur_A->sum_min = CUR_A_SUMMIN;
        Cur_A->output_max = CUR_A_OUTPUT_MAX;
        Cur_A->output_min = CUR_A_OUTPUT_MIN;
        
        Cur_B->Kp = CUR_A_KP;
        Cur_B->Ki = CUR_A_KI;
        Cur_B->Kd = CUR_A_KD;
        Cur_B->Kp_div = CUR_A_KP_DIV;
        Cur_B->Ki_div = CUR_A_KI_DIV; 
        Cur_B->Kd_div = CUR_A_KD_DIV;
        Cur_B->Kv_div = 1024;
        Cur_B->sum_max = CUR_A_SUMMAX;
        Cur_B->sum_min = CUR_A_SUMMIN;
        Cur_B->output_max = CUR_A_OUTPUT_MAX;
        Cur_B->output_min = CUR_A_OUTPUT_MIN;

        Ang->Kp = g_HLCmdMap[HL_PI_ANG_P];
        Ang->Ki = g_HLCmdMap[HL_PI_ANG_I];
        Ang->Kd = g_HLCmdMap[HL_PI_ANG_ERR_D];
        Ang->Kv = g_HLCmdMap[HL_PI_ANG_D];
        Ang->Kp_div = (float)g_HLCmdMap[HL_PI_ANG_P_DIV];              //float
        Ang->Ki_div = (float)g_HLCmdMap[HL_PI_ANG_I_DIV]; 
        Ang->Kd_div = (float)g_HLCmdMap[HL_PI_ANG_ERR_D_DIV];
        Ang->Kv_div = g_HLCmdMap[HL_PI_ANG_D_DIV];
        Ang->sum_max = ANGLE_A_SUMMAX;
        Ang->sum_min = ANGLE_A_SUMMIN;
        Ang->output_max = g_HLCmdMap[HL_PI_ANG_UPPER_OUTPUT_LMT];
        Ang->output_min = g_HLCmdMap[HL_PI_ANG_LOWER_OUTPUT_LMT];
        Ang->PI_area = g_HLCmdMap[HL_PI_ANG_I_AREA];
        Ang->PI_area_l = g_HLCmdMap[HL_PI_ANG_I_AREA_L];
        Ang->AREA_I = g_HLCmdMap[HL_PI_ANG_AREA_I];
        Ang->Err_D_Period = g_HLCmdMap[HL_PI_ANG_ERR_PERIOD];
        
        Prs->Kp = g_HLCmdMap[HL_PI_PRS_P];
        Prs->Ki = g_HLCmdMap[HL_PI_PRS_I];
        Prs->Kd = g_HLCmdMap[HL_PI_PRS_ERR_D];
        Prs->Kv = g_HLCmdMap[HL_PI_PRS_D];
        Prs->Kp_div = (float)g_HLCmdMap[HL_PI_PRS_P_DIV];
        Prs->Ki_div = (float)g_HLCmdMap[HL_PI_PRS_I_DIV];
        Prs->Kd_div = (float)g_HLCmdMap[HL_PI_PRS_ERR_D_DIV];
        Prs->Kv_div = g_HLCmdMap[HL_PI_PRS_D_DIV];
        Prs->sum_max =  PRS_A_SUMMAX;//(Prs->output_max * Prs->Ki_div / (Prs->Ki + 1));
        Prs->sum_min =  PRS_A_SUMMIN;//(Prs->output_min * Prs->Ki_div / (Prs->Ki + 1));
        Prs->output_max = g_HLCmdMap[HL_PI_PRS_UPPER_OUTPUT_LMT];
        Prs->output_min = g_HLCmdMap[HL_PI_PRS_LOWER_OUTPUT_LMT];
        Prs->PI_area = g_HLCmdMap[HL_PI_PRS_I_AREA];
        Prs->PI_area_l = g_HLCmdMap[HL_PI_PRS_I_AREA_L];
        Prs->AREA_I = g_HLCmdMap[HL_PI_PRS_AREA_I];
        Prs->Err_D_Period = g_HLCmdMap[HL_PI_PRS_ERR_PERIOD];
    }
    else
    {
        Cur_A->Kp = CUR_A_KP;
        Cur_A->Ki = CUR_A_KI;
        Cur_A->Kd = CUR_A_KD;
        Cur_A->Kp_div = CUR_A_KP_DIV;
        Cur_A->Ki_div = CUR_A_KI_DIV; 
        Cur_A->Kd_div = CUR_A_KD_DIV;
        Cur_A->Kv_div = 1024;
        Cur_A->sum_max = CUR_A_SUMMAX;
        Cur_A->sum_min = CUR_A_SUMMIN;
        Cur_A->output_max = CUR_A_OUTPUT_MAX;
        Cur_A->output_min = CUR_A_OUTPUT_MIN;
        
        Cur_B->Kp = CUR_A_KP;
        Cur_B->Ki = CUR_A_KI;
        Cur_B->Kd = CUR_A_KD;
        Cur_B->Kp_div = CUR_A_KP_DIV;
        Cur_B->Ki_div = CUR_A_KI_DIV; 
        Cur_B->Kd_div = CUR_A_KD_DIV;
        Cur_B->Kv_div = 1024;
        Cur_B->sum_max = CUR_A_SUMMAX;
        Cur_B->sum_min = CUR_A_SUMMIN;
        Cur_B->output_max = CUR_A_OUTPUT_MAX;
        Cur_B->output_min = CUR_A_OUTPUT_MIN;

        Ang->Kp = ANGLE_A_KP;
        Ang->Ki = ANGLE_A_KI;
        Ang->Kd = ANGLE_A_KD;
        Ang->Kv = ANGLE_A_KV;
        Ang->Kp_div = ANGLE_A_KP_DIV;
        Ang->Ki_div = ANGLE_A_KI_DIV; 
        Ang->Kd_div = ANGLE_A_KD_DIV;
        Ang->Kv_div = ANGLE_A_KV_DIV;
        Ang->sum_max = ANGLE_A_SUMMAX;
        Ang->sum_min = ANGLE_A_SUMMIN;
        Ang->output_max = ANGLE_A_OUTPUT_MAX;
        Ang->output_min = ANGLE_A_OUTPUT_MIN;
        Ang->PI_area = ANG_I_AREA;
        Ang->PI_area_l = ANG_I_AREA_L;
        Ang->AREA_I = ANG_AREA_I;
        Ang->Err_D_Period = ANG_ERR_D_PERIOD;
        
        Prs->Kp = PRS_A_KP;
        Prs->Ki = PRS_A_KI;
        Prs->Kd = PRS_A_KD;
        Prs->Kv = PRS_A_KV;
        Prs->Kp_div = PRS_A_KP_DIV;
        Prs->Ki_div = PRS_A_KI_DIV; 
        Prs->Kd_div = PRS_A_KD_DIV;
        Prs->Kv_div = PRS_A_KV_DIV;
        Prs->sum_max = PRS_A_SUMMAX;
        Prs->sum_min = PRS_A_SUMMIN;
        Prs->output_max = PRS_A_OUTPUT_MAX;
        Prs->output_min = PRS_A_OUTPUT_MIN;
        Prs->PI_area = ANG_I_AREA;
        Prs->PI_area_l = ANG_I_AREA_L;
        Prs->AREA_I = PRS_AREA_I;
        Prs->Err_D_Period = PRS_ERR_D_PERIOD;
    }
}

/*******************************************************************************
* Function Name : TiltAng_A_FreqShakeFunc
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u16 g_u16_AngSwinging_Freq_Angle = 400;
float g_f32_AngSwinging_Amp_Angle = 0.3;
void TiltAng_A_FreqShakeFunc(u32 *p)
{
        static u16 u16CycleCnt = 0;
        float f32ShakeAmpRatio = 0;         // 2% fluctuation amplitude
        u16 u16AngShakeCycle = 0;          // 1000ms
        
        u16AngShakeCycle = 10000 / g_u16_AngSwinging_Freq_Angle;
        f32ShakeAmpRatio = g_f32_AngSwinging_Amp_Angle;
        u16CycleCnt ++;
        if (u16CycleCnt <= u16AngShakeCycle / 2)
        {
            *p =  (u16)((*p) * (1.0f + f32ShakeAmpRatio));        
        }
        else if(u16CycleCnt > u16AngShakeCycle / 2)
        {
            if (u16CycleCnt == u16AngShakeCycle)
                u16CycleCnt = 0;
            
            *p =  (u16)((*p) * (1.0f - f32ShakeAmpRatio));
        }
        else{}     
}
/*******************************************************************************
* Function Name : TiltAng_B_FreqShakeFunc
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void TiltAng_B_FreqShakeFunc(u32 *p)
{
    #ifdef PPQ_1
        static u16 u16CycleCnt = 0;
        float f32ShakeAmpRatio = 0.1;         // 2% fluctuation amplitude
        u16 u16AngShakeCycle = 0;          // 1000ms
        
        u16AngShakeCycle = 10000 / User_Parameter.AngSwingPara->g_u16_AngSwinging_Freq;
        f32ShakeAmpRatio = User_Parameter.AngSwingPara->g_f32_AngSwinging_Amp;
        u16CycleCnt ++;
        if (u16CycleCnt <= u16AngShakeCycle / 2)
        {
            *p =  (u16)((*p) * (1.0f + f32ShakeAmpRatio));        
        }
        else if(u16CycleCnt > u16AngShakeCycle / 2)
        {
            if (u16CycleCnt == u16AngShakeCycle)
                u16CycleCnt = 0;
            
            *p =  (u16)((*p) * (1.0f - f32ShakeAmpRatio));
        }
        else{}
            
	#endif
}
   
/*******************************************************************************
* Function Name : TiltAngleLoop_Func
* Description   :
                    // Calculation formula: M = Vg * ��P / (20 * �� * Hydraulic system efficiency), where M is torque, Vg is displacement, ��P is pressure value, hydraulic efficiency is approximately 90%
* Input         :
* Output        :
* Return        :
*******************************************************************************/
#define LEAKAGECOMP_ENA         (0)                                                     // Leakage compensation
#define PRES_D_FDB_TO_ANG_ENA   (0)                                                     // Pressure differential compensation
#define POWER_LIMIT             (0)                                                     // Power limit
#define PRESSURE_LOOP           (0)

u8  g_u8_ExchangeFlg = 0;

u16 temple_min_cur_ref = 500;//490;
u16 temple_max_cur_ref = 1120;
u16	temple_t_u16_SolDeadCur_B = 830;
u16	temple_t_u16_SolDeadCur_Fall_B = 800;


float DeadZone = 500;

u16 Prs_set_offset = 0;

s16 g_s16_PrsCompAngVal = 0;
s16 g_u16_PrsLoop_P_Offset = 0;


u16 g_u16_TiltAngRef = 0;
u16  s_u16_TiltAngRef_PwrLmt = 0;

s16 g_s16_CurRef_PrsLoopOutput = 0;

s16 g_s16_PILoopMinOutput = 0;

s16 s16_PrsOffset = 0;
float Prs_offset_a = 0;
u16 Prs_offset_b = 0;

void TiltAngleLoop_Func()
{
    #ifdef PPQ_1
        u16 t_u16_SolDeadCur_A = 800;//656;
        u16 t_u16_Pump_Sol_A_Max_Cur_Ref = temple_max_cur_ref;//2358;                         
        u16 t_u16_Pump_Sol_A_Min_Cur_Ref = temple_min_cur_ref;
        u16 t_u16_Pump_Sol_B_Max_Cur_Ref = temple_max_cur_ref;//2447;
		u16 t_u16_Pump_Sol_B_Min_Cur_Ref = temple_min_cur_ref;//2447;
		u16 t_u16_SolDeadCur_B = temple_t_u16_SolDeadCur_B;
		u16 t_u16_SolDeadCur_B_Fall_B = temple_t_u16_SolDeadCur_Fall_B;
	
        static s16  s_s16_Cur_A_Ref, s_s16_Cur_B_Ref = 0;

	
    #endif   
    s16 t_s16_TiltAngRef;
    static s16 s_s16_AngLoopOutput_OffsetCur, s_s16_PrsLoopOutput_OffsetCur = 0;
	static s32 s_s32_OffsetCur = 0;
    static float s_f32_AngRef = 0;
	

    u16 t_u16_TiltAngRef, t_u16_TiltAngRef_PC = 0;
    float t_f32_Pump_CC = 0;



    /* Check if current is less than dead zone */
    if (User_Parameter.PpqEna->PumpStrt_Ena == 1)
    {
        /* Forward compensation of desired angle through pressure differential */
        g_s16_PrsCompAngVal = (s16)(User_Parameter.g_f32_PrsCompAngRatio * DetectorFdbVal.g_s32_PressureFdb_D);
        Limit_Q15(&g_s16_PrsCompAngVal, -2000, 2000);

        t_s16_TiltAngRef = (s16)(InstructionSet.TiltAngRef) - g_s16_PrsCompAngVal;   //100��Ϊ���ֲ���̬���

        // Leakage compensation
        if (User_Parameter.PpqEna->AngLeak_Ena)
        {
            t_s16_TiltAngRef += (s16)(t_s16_TiltAngRef * DetectorFdbVal.g_f32_PressureFdb * User_Parameter.g_f32_leakage_compensation);
        }
        Limit_Q15(&t_s16_TiltAngRef, 0, (short)(User_Parameter.SysPara->u16AngPerUnitScope + User_Parameter.g_u16_AngleLoop_P_Offset));
            
        t_u16_TiltAngRef_PC = (u16)t_s16_TiltAngRef;
        if (User_Parameter.PpqEna->PwrLoop_Ena)
        {
            if (DetectorFdbVal.g_f32_PressureFdb != 0)
            {
                t_f32_Pump_CC = InstructionSet.g_f32_TorqueLimt * User_Parameter.g_f32TorqueRatio / DetectorFdbVal.g_f32_PressureFdb;		//54.5867f			//���㹦�����Ƶ�����
                Limit_f32(&t_f32_Pump_CC, (float)User_Parameter.SysPara->u16PumpCC_MIN, (float)User_Parameter.SysPara->u16PumpCC_MAX);    
                s_u16_TiltAngRef_PwrLmt = (u16)((float)User_Parameter.SysPara->u16AngPerUnitScope * t_f32_Pump_CC / User_Parameter.SysPara->u16PumpCC_MAX);                          //ת����0~10000
			}
            else
            {
                s_u16_TiltAngRef_PwrLmt = User_Parameter.SysPara->u16AngPerUnitScope;
            }
            t_u16_TiltAngRef = s_u16_TiltAngRef_PwrLmt < t_u16_TiltAngRef_PC ? s_u16_TiltAngRef_PwrLmt : t_u16_TiltAngRef_PC;

            /* Filter the swing angle command */
            LowPassFilterFunc(&s_f32_AngRef, (float)t_u16_TiltAngRef, User_Parameter.LowPassFilterTimConst->g_f32_Pwr_Ref_FilterTimPara);//0.005f);
            t_u16_TiltAngRef = (u16)s_f32_AngRef;
        }
        else
        {
            LowPassFilterFunc(&s_f32_AngRef, (float)t_u16_TiltAngRef_PC, User_Parameter.LowPassFilterTimConst->g_f32_Ang_Ref_FilterTimPara);//0.005f); 
            t_u16_TiltAngRef = (u16)s_f32_AngRef;
            
            ScopeVal.g_u16_AngRef = (u16)s_f32_AngRef - User_Parameter.g_u16_AngleLoop_P_Offset;            
        }
    }
    else
    {
        s_f32_AngRef = 0;
    }
	
	
    if (User_Parameter.PpqEna->PumpStrt_Ena == 1)
    {
		if (User_Parameter.PpqEna->PrsLoop_Ena) 
		{
			s16_PrsOffset = InstructionSet.g_u16_PerUnitPrsVal * Prs_offset_a + Prs_offset_b;
            
			if (TimCnt.g_u16_Prs_PI_LOOP_Cnt == 0)
			{

				s_s16_PrsLoopOutput_OffsetCur = Pressure_PID_Regulator(InstructionSet.g_u16_PerUnitPrsVal+s16_PrsOffset,(u16)DetectorFdbVal.g_u32_PressureFdb, &Pressure_Loop);// Set value fixed offset

			}
			s_s16_PrsLoopOutput_OffsetCur = (s_s16_PrsLoopOutput_OffsetCur < t_u16_TiltAngRef) ? s_s16_PrsLoopOutput_OffsetCur : t_u16_TiltAngRef;

			if (TimCnt.g_u16_Ang_PI_LOOP_Cnt == 0)
			{
				s_s16_AngLoopOutput_OffsetCur = Angle_PID_Regulator(
					s_s16_PrsLoopOutput_OffsetCur,
					(u16)DetectorFdbVal.g_u32_AngFdb, 
					&Angle_Loop
				) + User_Parameter.g_u16_AngleLoop_P_Offset;
			}		
		}
		else
		{
			s_s16_PrsLoopOutput_OffsetCur = t_u16_TiltAngRef;
			// Angle loop uses original logic:
			if (TimCnt.g_u16_Ang_PI_LOOP_Cnt == 0)
			{
				s_s16_AngLoopOutput_OffsetCur = Angle_PID_Regulator(
					t_u16_TiltAngRef,
					(u16)DetectorFdbVal.g_u32_AngFdb, 
					&Angle_Loop
				) + User_Parameter.g_u16_AngleLoop_P_Offset;
			}			
		}


		s_s32_OffsetCur = s_s16_AngLoopOutput_OffsetCur;
        
   
		s_s16_Cur_B_Ref = Compute_Cur_B_Ref(s_s32_OffsetCur, DeadZone, t_u16_SolDeadCur_B, t_u16_SolDeadCur_B_Fall_B, t_u16_Pump_Sol_B_Max_Cur_Ref, t_u16_Pump_Sol_B_Min_Cur_Ref);

    }
    else
    {

        s_s16_PrsLoopOutput_OffsetCur = 0;
        s_s16_AngLoopOutput_OffsetCur = 0;
		
		#ifdef PPQ_1
		if (g_u8_ExchangeFlg == 1)
		{
			Rate_Tracking_Q15(&s_s16_Cur_B_Ref,(short)t_u16_Pump_Sol_B_Min_Cur_Ref,(short)100);
		}
		else if (g_u8_ExchangeFlg == 2)
		{
			Rate_Tracking_Q15(&s_s16_Cur_B_Ref,(short)t_u16_Pump_Sol_B_Max_Cur_Ref,(short)100);
		}
		else
		{
			Rate_Tracking_Q15(&s_s16_Cur_B_Ref,(short)0,(short)100);
		}
		#endif
	}
    /* ===============Current closed loop======================== */

    // Low-pass filter for AB current reference
    Cur_A_B_RefFilterFunc(s_s16_Cur_B_Ref);
    TiltAng_B_FreqShakeFunc(&g_u32_PWMOutput_B);
    pwm_output_A_B_process();


}

/*******************************************************************************
* Function Name : LimitAngLoopOutput
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u8 LimitAngLoopOutput(u16 TiltAngRef,u16 *CurRef,u16 MinCurRef,u16 MaxCurRef,u16 StrtCur)
{
    static u16 s_u16_500ms_DlyCnt = 0;
    static u16 s_u16_PreTiltAngleRef = 0;
    static u16 s_u16_PreCurRef = 0;
    static u8  s_u8_Flag = 0;
    
    if (TiltAngRef == 0)
    {
        s_u8_Flag = 0;
        s_u16_500ms_DlyCnt = 0;
        *CurRef = StrtCur;
        s_u16_PreTiltAngleRef = TiltAngRef;
        return (1);
    }
    
    if (s_u16_PreTiltAngleRef != TiltAngRef)
    {
        s_u16_500ms_DlyCnt = 0;
        s_u8_Flag = 1;
        s_u16_PreTiltAngleRef = TiltAngRef; 
    }
    
    if (s_u8_Flag == 1)
    {
       if (s_u16_500ms_DlyCnt <= 200)
       {
            s_u16_500ms_DlyCnt ++;
            // Limit output: limit increase, do not limit decrease, maintain output stability
            if (*CurRef > s_u16_PreCurRef)
            {
                if (*CurRef >= MaxCurRef)
                    *CurRef = MaxCurRef;
            }
            else if (*CurRef < s_u16_PreCurRef)
            {
                if (*CurRef <= MinCurRef)
                    *CurRef = MinCurRef;
            }
            else{}
                
            s_u16_PreCurRef = *CurRef;
       }
       else
       {
            s_u8_Flag = 0;
            s_u16_500ms_DlyCnt = 0;
            s_u16_PreCurRef = *CurRef;
       }
    }
    else
    {
        s_u8_Flag = 0;
        s_u16_500ms_DlyCnt = 0;
        s_u16_PreCurRef = *CurRef;        
    }
    return (1);
}
/*******************************************************************************
* Function Name : UpdateAngleLoopMaxMinErrSum
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u8 UpdateAngleLoopMaxMinErrSum(u16 TiltAngRef, u16 MinCurRef,u16 MaxCurRef,pid_location_t *Ang)
{
    static u16 s_u16_500ms_DlyCnt = 0;
    static u16 s_u16_PreTiltAngleRef = 0;
    static u8  s_u8_Flag = 0;

    if (TiltAngRef == 0)
    {
        s_u8_Flag = 0;
        s_u16_500ms_DlyCnt = 0;
        Ang->sum_max = ANGLE_A_OUTPUT_MAX * Ang->Ki_div / Ang->Ki;
        Ang->sum_min = ANGLE_A_OUTPUT_MIN * Ang->Ki_div / Ang->Ki;
        s_u16_PreTiltAngleRef = TiltAngRef;
        return (1);
    }
    
    if (s_u16_PreTiltAngleRef != TiltAngRef)
    {
        s_u16_500ms_DlyCnt = 0;
        s_u8_Flag = 1;
        s_u16_PreTiltAngleRef = TiltAngRef; 
    }

    if (s_u8_Flag == 1)
    {
        if (s_u16_500ms_DlyCnt <= 200)
        {
            s_u16_500ms_DlyCnt ++;
            Ang->sum_max = MaxCurRef * Ang->Ki_div / Ang->Ki;
            Ang->sum_min = MinCurRef * Ang->Ki_div / Ang->Ki;
        }
        else
        {
            s_u8_Flag = 0;
        }
    }
    else
    {
        s_u16_500ms_DlyCnt = 0;
        Ang->sum_max = ANGLE_A_OUTPUT_MAX * Ang->Ki_div / Ang->Ki;
        Ang->sum_min = ANGLE_A_OUTPUT_MIN * Ang->Ki_div / Ang->Ki;
    }
    
    return (1);
}
/*******************************************************************************
* Function Name : UpdateMapCmdData
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u8 READY = 0x01;
u8 CurFdb_A_BREAK = 0x01;
u8 CurFdb_B_BREAK = 0x01;
u8 ANGSENSOR_BREAK = 0x01;
u8 PRSSENSOR_01_BREAK = 0x01;
u8 PRSSENSOR_02_BREAK = 0x01;
u8 CHKE_BREAK = 0x01;
u8 E2PROM_BREAK = 0x01;
u8 BUS_BREAK = 0x01;

void UpdateMapCmdData(pid_location_t *Ang, pid_location_t *Prs)
{
    /* Unit data reading */
    g_PCRdCmd[HL_INF_SW_VERS] = (u32)SW_VERS;
    g_PCRdCmd[HL_INF_ANG_ADC_FDB] = (u32)ScopeVal.g_f32_Filter_SwivelAngle;    
    g_PCRdCmd[HL_INF_PRS_ADC_FDB] = (u32)ScopeVal.g_f32_Filter_PressureData;
    g_PCRdCmd[HL_INF_ANG_PER_UNIT] = (u32)DetectorFdbVal.g_u32_AngFdb;
    g_PCRdCmd[HL_INF_PRS_PER_UNIT] = (u32)DetectorFdbVal.g_u32_PressureFdb;
	g_PCRdCmd[HL_FDB_USB_ANG_REF] = (u32)InstructionSet.TiltAngRef;
	g_PCRdCmd[HL_FDB_USB_PRS_REF] = (u32)InstructionSet.g_u16_PerUnitPrsVal;
    g_PCRdCmd[HL_INF_PRS_VAL_BAR] = (u32)DetectorFdbVal.g_f32_PressureFdb*10;
	g_PCRdCmd[HL_INF_CUR_A_FDB] = (u32)DetectorFdbVal.g_s32_CurFdb_A;
    g_PCRdCmd[HL_INF_CUR_B_FDB] = (u32)DetectorFdbVal.g_s32_CurFdb_B;
	
	g_PCRdCmd[HL_INF_CC_MIN] = (u32)User_Parameter.SysPara->u16PumpCC_MIN;
    g_PCRdCmd[HL_INF_CC_MAX] = (u32)User_Parameter.SysPara->u16PumpCC_MAX;
    g_PCRdCmd[HL_INF_PI_ANG_P] = (u32)Ang->Kp;    
    g_PCRdCmd[HL_INF_PI_ANG_P_DIV] = (u32)Ang->Kp_div;
    g_PCRdCmd[HL_INF_PI_ANG_ERR_D] = (u32)Ang->Kd;
    g_PCRdCmd[HL_INF_PI_ANG_ERR_D_DIV] = (u32)Ang->Kd_div;
    g_PCRdCmd[HL_INF_PI_PRS_P] = (u32)Prs->Kp;
    g_PCRdCmd[HL_INF_PI_PRS_P_DIV] = (u32)Prs->Kp_div;
    g_PCRdCmd[HL_INF_PI_PRS_ERR_D] = (u32)Prs->Kd;
    g_PCRdCmd[HL_INF_PI_PRS_ERR_D_DIV] = (u32)Prs->Kd_div;
    
    g_PCRdCmd[HL_INF_ANG_MIN_ADC_FDB] = (u32)User_Parameter.SysPara->u16AngAdc_MIN;
	g_PCRdCmd[HL_INF_ANG_MID_ADC_FDB] = (u32)User_Parameter.SysPara->u16AngAdc_MID;
	g_PCRdCmd[HL_INF_ANG_MID_SCOPE_FDB] = (u32)User_Parameter.SysPara->u16AngAdc_MID_Scope;
    g_PCRdCmd[HL_INF_ANG_MAX_ADC_FDB] = (u32)User_Parameter.SysPara->u16AngAdc_MAX;
    g_PCRdCmd[HL_INF_PRS_MIN_ADC_FDB] = (u32)User_Parameter.SysPara->u16PrsAdc_MIN;
    g_PCRdCmd[HL_INF_PRS_MAX_ADC_FDB] = (u32)User_Parameter.SysPara->u16PrsAdc_MAX;
	
	g_PCRdCmd[HL_ERROR_READY] = (u32)READY;
	g_PCRdCmd[HL_ERROR_CurFdb_A_BREAK] = (u32)CurFdb_A_BREAK;
	g_PCRdCmd[HL_ERROR_CurFdb_B_BREAK] = (u32)CurFdb_B_BREAK;
	g_PCRdCmd[HL_ERROR_ANGSENSOR_BREAK] = (u32)ANGSENSOR_BREAK;
	g_PCRdCmd[HL_ERROR_PRSSENSOR_01_BREAK] = (u32)PRSSENSOR_01_BREAK;
	g_PCRdCmd[HL_ERROR_PRSSENSOR_02_BREAK] = (u32)PRSSENSOR_02_BREAK;
	g_PCRdCmd[HL_ERROR_CHKE_BREAK] = (u32)CHKE_BREAK;
	g_PCRdCmd[HL_ERROR_E2PROM_BREAK] = (u32)E2PROM_BREAK;
	g_PCRdCmd[HL_ERROR_BUS_BREAK] = (u32)BUS_BREAK;
	
	g_PCSetCmd[HL_ENA_PUMP_STRT] = (u16)User_Parameter.PpqEna->PumpStrt_Ena;
	g_PCSetCmd[HL_ENA_PRS_LOOP]  = (u16)User_Parameter.PpqEna->PrsLoop_Ena;
	g_PCSetCmd[HL_ENA_PWR_LOOP]  = (u16)User_Parameter.PpqEna->PwrLoop_Ena;


}
/*******************************************************************************
* Function Name : USB_PackWRBackDataArray
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Update_CtrlPara()
{
    User_Parameter.PpqEna->AngLeak_Ena = g_HLCmdMap[HL_ENA_LEAKEAGE];
    
    User_Parameter.TimPeriod->g_u16_Ang_PI_LOOP_Period = g_HLCmdMap[HL_PRD_ANG_LOOP];
    User_Parameter.TimPeriod->g_u16_Prs_PI_LOOP_Period = g_HLCmdMap[HL_PRD_PRS_LOOP];
    User_Parameter.TimPeriod->g_u16_Angle_Err_D_Period = g_HLCmdMap[HL_PRD_ANG_ERR_D];
    User_Parameter.TimPeriod->g_u16_Prs_Err_D_Period = g_HLCmdMap[HL_PRD_PRS_ERR_D];
    User_Parameter.TimPeriod->g_u16_Ang_Fdb_D_Period = g_HLCmdMap[HL_PRD_ANG_FDB_D];
    User_Parameter.TimPeriod->g_u16_Prs_Fdb_D_Period = g_HLCmdMap[HL_PRD_PRS_FDB_D];

    /* Note: This is a floating point number, pay attention to precision */
    User_Parameter.LowPassFilterTimConst->g_f32_Ang_Ref_FilterTimPara = (float)g_HLCmdMap[HL_TIM_C_ANG_REF_FILTER] / 10000;
    User_Parameter.LowPassFilterTimConst->g_f32_Ang_Fdb_FilterTimPara = (float)g_HLCmdMap[HL_TIM_C_ANG_FDB_FILTER] / 10000;
    User_Parameter.LowPassFilterTimConst->g_f32_Prs_Fdb_FilterTimPara = (float)g_HLCmdMap[HL_TIM_C_PRS_FDB_FILTER] / 10000;
    
    /**/
    User_Parameter.AngSwingPara->g_f32_AngSwinging_Amp  = (float)g_HLCmdMap[HL_SWING_AMP] / 10000;
    User_Parameter.AngSwingPara->g_u16_AngSwinging_Freq = g_HLCmdMap[HL_SWING_FREQ];
    
    /**/
    User_Parameter.g_u16_AngleLoop_P_Offset = g_HLCmdMap[HL_CTRL_ANG_P_COMP_RATIO];
    User_Parameter.g_f32_PrsCompAngRatio = (float)g_HLCmdMap[HL_CTRL_PRS_COMP_ANG_RATIO] / 10000;
    User_Parameter.g_f32_leakage_compensation = (float)g_HLCmdMap[HL_CTRL_LKGE_COMP_RATIO] / 10000;
    
    /**/
    User_Parameter.SysPara->u16AngAdc_MIN = g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MIN];
	User_Parameter.SysPara->u16AngAdc_MID = g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MID];
	User_Parameter.SysPara->u16AngAdc_MID_Scope = g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MID_SCOPE];
    User_Parameter.SysPara->u16AngAdc_MAX = g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MAX];
    User_Parameter.SysPara->u16AngPerUnitScope = g_HLCmdMap[HL_SYS_PARA_ANG_ADC_SCOPE];
    User_Parameter.SysPara->u16PrsAdc_MIN = g_HLCmdMap[HL_SYS_PARA_PRS_ADC_MIN];
    User_Parameter.SysPara->u16PrsAdc_MAX = g_HLCmdMap[HL_SYS_PARA_PRS_ADC_MAX];
    User_Parameter.SysPara->u16PumpCC_MIN = g_HLCmdMap[HL_SYS_PARA_CC_MIN];
    User_Parameter.SysPara->u16PumpCC_MAX = g_HLCmdMap[HL_SYS_PARA_CC_MAX];
    User_Parameter.SysPara->u16Valve_4mA_Dac = g_HLCmdMap[HL_SYS_PARA_DAC_OUT_MIN];
    User_Parameter.SysPara->u16Valve_20mA_Dac = g_HLCmdMap[HL_SYS_PARA_DAC_OUT_MAX];
    User_Parameter.SysPara->u16Valve_MidPosi_Dac = g_HLCmdMap[HL_SYS_PARA_DAC_OUT_MID];
	
}

