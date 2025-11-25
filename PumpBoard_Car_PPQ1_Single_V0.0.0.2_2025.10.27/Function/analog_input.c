#include "stdlib.h"
#include "algorithm.h"
#include "application.h"  
#include "AD7689.h"
#include "Hw_spi.h"
#include "Hw_ADC.h"

_ANALOG_INPUT_VALUE     AnalogInput = _ANALOG_INPUT_VALUE_DEFAULT;
_FEEDBACK_VALUE         DetectorFdbVal = _FEEDBACK_VALUE_DEFAULT;
volatile u16 g_ADC_Original_Value[MCU_ADC_CHANNEL_NUM * MCU_ADC_AVG_NUM] = {0};

/*******************************************************************************
* Function Name : GetOriginalADCVal()
* Description   : AD7949数据求平均                
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u16 GetOriginalADCVal(u16 Channel)
{
    u8 result = 0;
    u16 t_u16RecvOriginalADCVal = 0;
    
    u16 AD7689_CFG_Reg = CFG_OVERWRITE | INCC_UNIPOLAR_TO_COM | Channel | BW_FULL | REF_IN_2V5 | SEQ_DISABLE | RB_DISABLE;
    
    AD7689_CFG_Reg = AD7689_CFG_Reg << 0x02;
    result = SPI_4WriteByte(AD7689_CFG_Reg, &t_u16RecvOriginalADCVal);
    
    return (t_u16RecvOriginalADCVal);
}
/*******************************************************************************
* Function Name : Cal_MCU_AvgAdcVal_Fun
* Description   : 计算MCU自身AD采样平均数据   
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Cal_MCU_AvgAdcVal_Fun()
{
    u8  i , j = 0;
    u32 Sum[MCU_ADC_CHANNEL_NUM] = {0};
    
    for (j = 0; j < MCU_ADC_AVG_NUM; j ++)
    {
        for (i = 0; i < MCU_ADC_CHANNEL_NUM; i ++)
        {
            Sum[i] += g_ADC_Original_Value[i + j * MCU_ADC_CHANNEL_NUM];
        }
    }
    for (i = 0; i < MCU_ADC_CHANNEL_NUM; i ++)
    {
        if (i == 0)
            AnalogInput.g_u16_SENSOR_OVER_CURRENT = Sum[i] / MCU_ADC_AVG_NUM;
        else if (i == 1)
            AnalogInput.g_u16_ANALOG_01_OUTPUT_CHECK = Sum[i] / MCU_ADC_AVG_NUM;
        else if (i == 2)
            AnalogInput.g_u16_ANALOG_02_OUTPUT_CHECK = Sum[i] / MCU_ADC_AVG_NUM;
        else if (i == 3)
            AnalogInput.g_u16_CURRENT_A_CHECK = Sum[i] / MCU_ADC_AVG_NUM;
        else if (i == 4)
            AnalogInput.g_u16_CURRENT_B_CHECK = Sum[i] / MCU_ADC_AVG_NUM;
        else{}
    }
}
/*******************************************************************************
* Function Name : LowPassFilterFunc
* Description   : 一阶低通滤波（暂时未用）
                    a单位是us
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void LowPassFilterFunc(float *Pre, float p, float a)
{
//    static s32 s_s32_PreADCVal = 0;
//    u32 *p1;
//    *p1 =  ((1000000 - a) * (*Pre) + a * (*p)) / 1000000;
//    *Pre = *p1;
    *Pre =  (1 - a) * (*Pre) + a * (p);
}

//void LowPassFilterFunc_Float(float *Pre, float p, float a)
//{
////    static s32 s_s32_PreADCVal = 0;
////    float *p1;
//    
////    *Pre =  ((1000000 - a) * (*Pre) + a * (p)) / 1000000;
//    *Pre =  (1 - a) * (*Pre) + a * (p);
////    s_u32_Pre_TestLowPassFilter = p1;
//}
/*******************************************************************************
* Function Name : Cal_AngFbkVal_Func
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Cal_AngFbkVal_Func(float *p1)
{
    u16 t_u16_MinAdcAngVal = 0;
	u16 t_u16_MidAdcAngVal = 0;
	u16 t_u16_MidAdcAngVal_Scope = 0;
    u16 t_u16_MaxAdcAngVal = 0;
    u32 t_u32_SwivelAngle = (u32)*p1;
    u32 t_u32_Scope        = 0;
    t_u16_MinAdcAngVal = User_Parameter.SysPara->u16AngAdc_MIN;
	t_u16_MidAdcAngVal = User_Parameter.SysPara->u16AngAdc_MID;
	t_u16_MidAdcAngVal_Scope = User_Parameter.SysPara->u16AngAdc_MID_Scope;
    t_u16_MaxAdcAngVal = User_Parameter.SysPara->u16AngAdc_MAX;
    t_u32_Scope = (u32)User_Parameter.SysPara->u16AngPerUnitScope;
    

	  DetectorFdbVal.g_u32_AngFdb = FeedbackSignalPerUnitFunc_AngMID(t_u32_SwivelAngle,t_u16_MidAdcAngVal,t_u16_MaxAdcAngVal,t_u16_MidAdcAngVal_Scope);
//    DetectorFdbVal.g_u32_AngFdb = FeedbackSignalPerUnitFunc(t_u32_SwivelAngle,t_u32_Scope,t_u16_MinAdcAngVal,t_u16_MaxAdcAngVal);
    
//    DetectorFdbVal.g_u32_AngFdb = t_u32_Scope - FeedbackSignalPerUnitFunc(t_u32_SwivelAngle,t_u32_Scope,t_u16_MinAdcAngVal,t_u16_MaxAdcAngVal);
}
/*******************************************************************************
* Function Name : Cal_PrsFbkVal_Func
* Description   : 单位是Bar
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Cal_PrsFbkVal_Func(float *p1)
{
    u16 t_u16_MinAdcPrsVal = 0;
	u16 t_u16_MidAdcPrsVal = 0;
	u16 t_u16_MidAdcPrsVal_Scope = 0;
    u16 t_u16_MaxAdcPrsVal = 0;
	u16 t_u16_MaxAdcPrsVal_Scope =0;
    u16 t_u32_PumpPrs = (u16)*p1;
    u32 t_u32_Scope        = 5000;
    
    t_u16_MinAdcPrsVal = User_Parameter.SysPara->u16PrsAdc_MIN;
	t_u16_MidAdcPrsVal = User_Parameter.SysPara->u16PrsAdc_MID;
	t_u16_MidAdcPrsVal_Scope = User_Parameter.SysPara->u16PrsAdc_MID_Scope;//这里的scope是bar
    t_u16_MaxAdcPrsVal = User_Parameter.SysPara->u16PrsAdc_MAX;
	t_u16_MaxAdcPrsVal_Scope = User_Parameter.SysPara->u16PrsAdc_MAX_Scope;//这里的scope是bar
    
//    DetectorFdbVal.g_f32_PressureFdb = 0.020f * (float)(AnalogInput.g_u16_IN0_Pressure_01) - 15.49f;
//    DetectorFdbVal.g_f32_PressureFdb =  (float)((t_u16_MaxAdcPrsVal_Scope - t_u16_MidAdcPrsVal_Scope)/(t_u16_MaxAdcPrsVal- t_u16_MidAdcPrsVal)) * (*p1 - t_u16_MidAdcPrsVal) + t_u16_MidAdcPrsVal_Scope;// + 7.4f;
	DetectorFdbVal.g_f32_PressureFdb =  0.0171f * (*p1 - t_u16_MidAdcPrsVal);// + 7.4f;
    DetectorFdbVal.g_f32_PressureFdb = 	DetectorFdbVal.g_f32_PressureFdb <= 0 ? 0 : DetectorFdbVal.g_f32_PressureFdb;
    
    DetectorFdbVal.g_u32_PressureFdb = (u32)(FeedbackSignalPerUnitFunc(t_u32_PumpPrs, t_u16_MidAdcPrsVal,t_u16_MaxAdcPrsVal,t_u16_MidAdcPrsVal_Scope));
}
/*******************************************************************************
* Function Name : Cal_AngFbkDiff_Func
* Description   : 角度反馈微分计算
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Cal_AngFbkDiff_Func()
{
    static  s32 s_s32_Pre_AngFdb = 0;
    
    if (User_Parameter.PpqEna->PumpStrt_Ena == 1)
    {
        if (TimCnt.g_u16_Ang_Fdb_D_Cnt == 0)                 
        {   
            DetectorFdbVal.g_s32_AngFdb_D = (s32)DetectorFdbVal.g_u32_AngFdb - s_s32_Pre_AngFdb;//       / (ANG_D_FILTER_CNT * 100); 
            Angle_Loop.g_s16_Kv_Err = (s16)DetectorFdbVal.g_s32_AngFdb_D; 
            s_s32_Pre_AngFdb = (s32)DetectorFdbVal.g_u32_AngFdb;
        }
    }
    else
    {
        DetectorFdbVal.g_s32_AngFdb_D = 0;
        Angle_Loop.g_s16_Kv_Err = 0;
        s_s32_Pre_AngFdb = (s32)DetectorFdbVal.g_u32_AngFdb;
    } 
}
/*******************************************************************************
* Function Name : CalAngFbkDiff_InPrsLoop_Func
* Description   : 该角度微分用于压力环的前馈计算
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u16 g_u16_Ang_D_Cnt_InPrsLoop = 0;
u16 g_u16_Ang_D_TimCnt_InPrsLoop = 50;
void CalAngFbkDiff_InPrsLoop_Func()
{
    static  s32 s_s32_Pre_AngFdb = 0;
    
    if (User_Parameter.PpqEna->PumpStrt_Ena == 1)
    {
        if (g_u16_Ang_D_Cnt_InPrsLoop ++ >= g_u16_Ang_D_TimCnt_InPrsLoop)                 
        {   
            Pressure_Loop.g_s16_AngInPrs_Kv_Err = (s16)((s32)DetectorFdbVal.g_u32_AngFdb - s_s32_Pre_AngFdb);//       / (ANG_D_FILTER_CNT * 100); 
//            Angle_Loop.g_s16_Kv_Err = (s16)DetectorFdbVal.g_s32_AngFdb_D; 
            s_s32_Pre_AngFdb = (s32)DetectorFdbVal.g_u32_AngFdb;
        }
    }
    else
    {
        Pressure_Loop.g_s16_AngInPrs_Kv_Err = 0;
        s_s32_Pre_AngFdb = (s32)DetectorFdbVal.g_u32_AngFdb;
        g_u16_Ang_D_Cnt_InPrsLoop = 0;
    } 
}
/*******************************************************************************
* Function Name : Cal_PrsFbkDiff_Func
* Description   : 压力反馈微分计算
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Cal_PrsFbkDiff_Func()
{
    static  s32 s_s32_Pre_PressureFdb = 0;
    
    if (User_Parameter.PpqEna->PumpStrt_Ena == 1)
    {
        if (TimCnt.g_u16_Prs_Fdb_D_Cnt == 0 )
        {
            DetectorFdbVal.g_s32_PressureFdb_D = (s32)DetectorFdbVal.g_u32_PressureFdb - s_s32_Pre_PressureFdb;           
            Pressure_Loop.g_s16_Kv_Err = (s16)DetectorFdbVal.g_s32_PressureFdb_D;            //10000是指100%基础上放大100倍
            s_s32_Pre_PressureFdb = (s32)DetectorFdbVal.g_u32_PressureFdb;
        }

    }
    else
    {
        Pressure_Loop.g_s16_Kv_Err = 0;
        DetectorFdbVal.g_s32_PressureFdb_D = 0;
        s_s32_Pre_PressureFdb = (s32)DetectorFdbVal.g_u32_PressureFdb;
    }
}
/*******************************************************************************
* Function Name : CalAD7949AvgAngPrsData
* Description   :                   
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
//float g_f32_AngFilterPara = ANG_FDB_FILTER_TIM_CONST;//                              调整该值，会反过来影响其超调比例，这是因为需要考虑相位延迟，幅值衰减等因素
//float g_f32_Prs_Fdb_FilterPara = PRS_FDB_FILTER_TIM_CONST;//0.005f
void CalAD7689AvgAngPrsData()           //函数执行周期100us
{
    static float s_f32_Pre_Pressure_01 = 0;
    static float s_f32_Pre_Pressure_02 = 0;
    static float s_f32_Pre_SwivelAngle_01 = 0;
	static float s_f32_Pre_SwivelAngle_02 = 0;
    /*一阶滤波*/

    LowPassFilterFunc(&s_f32_Pre_Pressure_01, (float)AnalogInput.g_u32_Pressure_01, User_Parameter.LowPassFilterTimConst->g_f32_Prs_Fdb_FilterTimPara);//PRS_FILTER_100us_CNT * 100);         //100是指100us
    LowPassFilterFunc(&s_f32_Pre_Pressure_02, (float)AnalogInput.g_u32_Pressure_02, User_Parameter.LowPassFilterTimConst->g_f32_Prs_Fdb_FilterTimPara);//PRS_FILTER_100us_CNT * 100);
    ScopeVal.g_f32_Filter_PressureData = s_f32_Pre_Pressure_01;
    /*实际压力值计算*/
    Cal_PrsFbkVal_Func(&s_f32_Pre_Pressure_01);    

    LowPassFilterFunc(&s_f32_Pre_SwivelAngle_01, (float)AnalogInput.g_u32_SwivelAngle_01, User_Parameter.LowPassFilterTimConst->g_f32_Ang_Fdb_FilterTimPara);//ANG_FILTER_100us_CNT * 100);
	LowPassFilterFunc(&s_f32_Pre_SwivelAngle_02, (float)AnalogInput.g_u32_SwivelAngle_02, User_Parameter.LowPassFilterTimConst->g_f32_Ang_Fdb_FilterTimPara);//ANG_FILTER_100us_CNT * 100);
    ScopeVal.g_f32_Filter_SwivelAngle = s_f32_Pre_SwivelAngle_02;        
    /*实际角度值计算--标幺至[0,10000]*/
    Cal_AngFbkVal_Func(&s_f32_Pre_SwivelAngle_02);    

}
/*******************************************************************************
* Function Name : Cur_A_B_RefFilterFunc
* Description   :                   
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
float g_f32_CurFilterPara = 0.083f;

float s_u32_Pre_TestLowPassFilter = 0;
float s_u32_TestLowPassFilter = 0;
float s_u32_TestLowPassFilterPara = 0.005f;

void Cur_A_B_RefFilterFunc(s16 Cur_B)
{
    static float s_f32_Pre_Cur_B = 0;
 
    LowPassFilterFunc(&s_f32_Pre_Cur_B, (float)Cur_B, g_f32_CurFilterPara);

    InstructionSet.Cur_B_Ref = (u32)s_f32_Pre_Cur_B;        

}

void Test_LowPassFilterFunc()
{
    LowPassFilterFunc(&s_u32_Pre_TestLowPassFilter, s_u32_TestLowPassFilter, s_u32_TestLowPassFilterPara);
}
/*******************************************************************************
* Function Name : TransAD7949DataIntoFIFO
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void TransAD7689DataIntoFIFO()
{
    u8  i = 0;
        for (i = 0; i < ADC_CHANNEL_NUM; i ++)             //8通道,耗时28us,当前配置写入，输出的是往前数2轮的数据
        {
            if (i == 0)
            {
                AnalogInput.g_u32_Pressure_01 = GetOriginalADCVal(IN0);
            }
            else if (i == 1)
            {
                AnalogInput.g_u32_SwivelAngle_01 = GetOriginalADCVal(IN1);
            }
            else if (i == 2)
            {
                AnalogInput.g_u32_Vbus = GetOriginalADCVal(IN2);
            }
			else if (i == 3)
            {
                AnalogInput.g_u32_CURRENT_A = GetOriginalADCVal(IN3);
            }
            else if (i == 4)
            {
                AnalogInput.g_u32_CURRENT_B = GetOriginalADCVal(IN4);
            }
            else if (i == 5)
            {
                AnalogInput.g_u32_NONEED = GetOriginalADCVal(IN5);
            }
			else if (i == 6)
            {
                AnalogInput.g_u32_Pressure_02 = GetOriginalADCVal(IN6);
            }
			else if (i == 7)
            {
                AnalogInput.g_u32_SwivelAngle_02 = GetOriginalADCVal(IN7);
            }
			else{}
        }
}
/*******************************************************************************
* Function Name : Cal_AD7949_AvgAdcVal_Fun
* Description   : AD7949采样传输周期100us;   计算AD7949芯片采集所得的平均数据
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
//extern u16 g_u16AD7949_rxBuf[ADC_CHANNEL_NUM * ADC_FILTER_FIFO];
//void Cal_AD7949_AvgAdcVal_Fun()
//{
//    u8  i , j = 0;
//    u32 Sum[ADC_CHANNEL_NUM] = {0};
//    
//    for (j = 0; j < ADC_FILTER_FIFO; j ++)
//    {
//        for (i = 0; i < ADC_CHANNEL_NUM; i ++)
//        {
//            Sum[i] += g_u16AD7949_rxBuf[i + j * ADC_CHANNEL_NUM];
//        }
//    }
//    for (i = 0; i < ADC_CHANNEL_NUM; i ++)
//    {
//        if (i == 0)
//            AnalogInput.g_u16_IN0_Pressure_01 = Sum[i] / ADC_FILTER_FIFO;
//        else if (i == 1)
//            AnalogInput.g_u16_IN1_Pressure_02 = Sum[i] / ADC_FILTER_FIFO;
//        else if (i == 2)
//            AnalogInput.g_u16_IN2_SwivelAngle = Sum[i] / ADC_FILTER_FIFO;
//        else if (i == 3)
//            AnalogInput.g_u16_IN3_Output_5V = Sum[i] / ADC_FILTER_FIFO;
//        else{}
//    }
//}
