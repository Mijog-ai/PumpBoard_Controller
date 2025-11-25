/********************************************************************************************************
*	@file 	    Diagnose.c
*	@brief
*	@author     
*	@version    
*	@date       
*	@warning
*********************************************************************************************************/
#include "Diagnose.h"
#include "application.h"  
#include "gpio.h"

/******************************************* variable define ********************************************/
u8  g_u8_ErrorByte = 0;
/******************************************* function define ********************************************/


/*********************************************************************************************************
 * FunctionName   : DiagnoseCurFdbASensorErr()
 * Description    : 诊断故障
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void DiagnoseCurFdbASensorErr(u8 ErrByte)
{
    static u16 s_u16_SetErrDlyCnt = 0;
    static u16 s_u16_ClrErrDlyCnt = 0;
    static u8 pe12Status = 0;
	pe12Status = GetPe12Status();
	
    if (pe12Status == 0)  // PE12为低电平，表示发生错误
    {
		s_u16_SetErrDlyCnt++;
        if (s_u16_SetErrDlyCnt > 20)  // 滤波1秒时长
        {
            s_u16_SetErrDlyCnt = 0;
            SetErrorByte(ErrByte);  // 设置错误
        }
    }
    else  // PE12为低电平，清除错误
    {
        s_u16_ClrErrDlyCnt++;
        if (s_u16_ClrErrDlyCnt > 20)
        {
            s_u16_ClrErrDlyCnt = 0;
            ClearErrorByte(ErrByte);  // 清除错误
        }
    }
}

/*********************************************************************************************************
 * FunctionName   : DiagnoseCurFdbBSensorErr()
 * Description    : 诊断故障
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void DiagnoseCurFdbBSensorErr(u8 ErrByte)
{
    static u16 s_u16_SetErrDlyCnt = 0;
    static u16 s_u16_ClrErrDlyCnt = 0;
    static u8 pd10Status = 0;
	pd10Status = GetPd10Status();
    if (pd10Status == 0)  // PD10为低电平，表示发生错误
    {
        s_u16_SetErrDlyCnt++;
        if (s_u16_SetErrDlyCnt > 20)  // 滤波1秒时长
        {
            s_u16_SetErrDlyCnt = 0;
            SetErrorByte(ErrByte);  // 设置错误
        }
    }
    else  // PD10为低电平，清除错误
    {
        s_u16_ClrErrDlyCnt++;
        if (s_u16_ClrErrDlyCnt > 20)
        {
            s_u16_ClrErrDlyCnt = 0;
            ClearErrorByte(ErrByte);  // 清除错误
        }
    }
}
/*********************************************************************************************************
 * FunctionName   : DiagnosePressure01SensorErr()
 * Description    : 诊断故障
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void DiagnosePressure01SensorErr(u32 p, u32 Min, u32 Max, u8 ErrByte)
{
    static u16  s_u16_SetErrDlyCnt = 0;
    static u16  s_u16_ClrErrDlyCnt = 0;
    u32 t_u32_MaxUnsignedshort = 0xFFFF;
    
    if (t_u32_MaxUnsignedshort > Max)
        t_u32_MaxUnsignedshort = Max;
        
    if ((p <= Min) || (p >= t_u32_MaxUnsignedshort))	
	{
		s_u16_SetErrDlyCnt ++;		
		if (s_u16_SetErrDlyCnt > 200)         //滤1S时长
		{
			s_u16_SetErrDlyCnt = 0;
			s_u16_ClrErrDlyCnt = 0;
			SetErrorByte(ErrByte);
			
		}
		else
		{}
	}
    else
    {       
        if (s_u16_ClrErrDlyCnt <= 200)
        {
            s_u16_ClrErrDlyCnt ++;
        }
        else
        {
            s_u16_SetErrDlyCnt = 0;
            ClearErrorByte(ErrByte);
        }
    }
}

/*********************************************************************************************************
 * FunctionName   : DiagnosePressure02SensorErr()
 * Description    : 诊断故障
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void DiagnosePressure02SensorErr(u32 p, u32 Min, u32 Max, u8 ErrByte)
{
    static u16  s_u16_SetErrDlyCnt = 0;
    static u16  s_u16_ClrErrDlyCnt = 0;
    u32 t_u32_MaxUnsignedshort = 0xFFFF;
    
    if (t_u32_MaxUnsignedshort > Max)
        t_u32_MaxUnsignedshort = Max;
        
    if ((p <= Min) || (p >= t_u32_MaxUnsignedshort))	
	{
		s_u16_SetErrDlyCnt ++;		
		if (s_u16_SetErrDlyCnt > 200)         //滤1S时长
		{
			s_u16_SetErrDlyCnt = 0;
			s_u16_ClrErrDlyCnt = 0;
			SetErrorByte(ErrByte);
		}
		else
		{}
	}
    else
    {       
        if (s_u16_ClrErrDlyCnt <= 200)
        {
            s_u16_ClrErrDlyCnt ++;
        }
        else
        {
            s_u16_SetErrDlyCnt = 0;
            ClearErrorByte(ErrByte);
        }
    }
}
/*********************************************************************************************************
 * FunctionName   : DiagnoseAngleSensorErr()
 * Description    : 诊断故障
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void DiagnoseAngleSensorErr(u32 p, u32 Min, u32 Max, u8 ErrByte)
{
    static u16  s_u16_SetErrDlyCnt = 0;
    static u16  s_u16_ClrErrDlyCnt = 0;
    u32 t_u32_MaxUnsignedshort = 0xFFFF;
    
    if (t_u32_MaxUnsignedshort > Max)
        t_u32_MaxUnsignedshort = Max;
        
    if ((p <= Min) || (p >= t_u32_MaxUnsignedshort))	
	{
		s_u16_SetErrDlyCnt ++;		
		if (s_u16_SetErrDlyCnt > 200)         //滤1S时长
		{
			s_u16_SetErrDlyCnt = 0;
			s_u16_ClrErrDlyCnt = 0;
			SetErrorByte(ErrByte);
		}
		else
		{}
	}
    else
    {       
        if (s_u16_ClrErrDlyCnt <= 200)
        {
            s_u16_ClrErrDlyCnt ++;
        }
        else
        {
            s_u16_SetErrDlyCnt = 0;
            ClearErrorByte(ErrByte);
        }
    }
}

uint8_t GetPe12Status(void)
{
    // 读取PD10的电平状态
    uint8_t pe12Status = (GPIOE->IDR >> 12) & 0x01;

    return pe12Status;
}

uint8_t GetPd10Status(void)
{
    // 读取PD10的电平状态
    uint8_t pd10Status = (GPIOD->IDR >> 10) & 0x01;

    return pd10Status;
}
/*********************************************************************************************************
 * FunctionName   : SetErrorByte()
 * Description    : 
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void SetErrorByte(u8 ErrorByte)
{
	g_u8_ErrorByte |= ErrorByte;
	
}
/*********************************************************************************************************
 * FunctionName   : ClearErrorByte()
 * Description    : 
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void ClearErrorByte(u8 ErrorByte)
{
	g_u8_ErrorByte &= ~ErrorByte;
	
}
/*************  COPYRIGHT (C) 2024 ******************* END OF FILE **************************************/
