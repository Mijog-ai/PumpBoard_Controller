/********************************************************************************************************
*	@file 	    Diagnose.h
*	@brief
*	@author     Peng.Zhang
*	@version    
*	@date       2024.10.04
*	@warning
*********************************************************************************************************/

/****************************** Define to prevent recursive inclusion **********************************/
#ifndef _DIAGNOSE_H
#define _DIAGNOSE_H
#include <stdint.h>
#include "Hw_ADC.h"
/******************************************* Macro define ********************************************/
#define PUMP_ERROR_CurFdb_A_BREAK                      	0x01
#define PUMP_ERROR_CurFdb_B_BREAK                      	0x02	
#define PUMP_ERROR_ANGSENSOR_BREAK                      0x04
#define PUMP_ERROR_PRSSENSOR_01_BREAK                   0x08
#define PUMP_ERROR_PRSSENSOR_02_BREAK                   0x10
#define PROFINET_ERROR_CHKE_BREAK                   	0x20
#define CTRLER_ERROR_E2PROM_BREAK                   	0x40
#define PROFINET_ERROR_BUS_BREAK                   		0x80
/******************************************* variable define ********************************************/
 

/******************************************* function define ********************************************/
void DiagnoseCurFdbASensorErr(u8);
void DiagnoseCurFdbBSensorErr(u8);
void DiagnoseAngleSensorErr(u32, u32, u32, u8);
void DiagnosePressure01SensorErr(u32, u32, u32, u8);
void DiagnosePressure02SensorErr(u32, u32, u32, u8);
uint8_t GetPe12Status(void);
uint8_t GetPd10Status(void);
void SetErrorByte(u8);
void ClearErrorByte(u8);


#endif/* _DIAGNOSE_H */
/*************  COPYRIGHT (C) 2019 ******************* END OF FILE **************************************/
