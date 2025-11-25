#include "application.h"
#include "Diagnose.h"
//ota_t ota;
//u32 debug_flag;
//u16 data_buff[PARAMETER_MAX_COUNT];
const uint32_t Application_Valid_Flag __attribute__((section(".ARM.__at_0x0803C000")))=0xAC3553CA;

/*******************************************************************************
* Function Name : Flash_Read()
* Description   :       
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Flash_Read(u32 Address, u8* Readbuff, u32 Len)
{
	s32 i;
	for(i=0; i<Len; i++)
	{ 
       Readbuff[i] = ((vu8*)Address)[i]; 
    }		
} // */

//Read halfword at specified address (16-bit data)
//faddr: read address
//Return value: corresponding data
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
} 

//Return value: corresponding data
u16 STMFLASH_Read_HalfWord(u32 faddr)
{
	return *(vu32*)faddr; 
} 
//Get flash sector where address is located
//addr: flash address
//Return value: 0~11, sector where addr is located
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
    else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	return FLASH_Sector_11;	
}
//Write specified length of data starting from specified address
//Important note: Because STM32F4 sector is too large, cannot guarantee save operations, therefore need to ensure
//         If write address is not 0xFF, must erase sector first or it will cause previous data loss
//         Writing to non-0xFF addresses will cause other sector data loss. Must ensure before writing
//         No required data, or backup entire sector data, erase, then rewrite
//This function is also valid for OTP area! But be cautious writing OTP!
//OTP area address range: 0X1FFF7800~0X1FFF7A0F
//WriteAddr: start address (this address must be multiple of 4!!)
//pBuffer: data pointer
//NumToWrite: word (32-bit) count (number of 32-bit data to write) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//Illegal address
	FLASH_Unlock();									//Unlock
  FLASH_DataCacheCmd(DISABLE);//During FLASH erase, must disable data cache
 		
	addrx=WriteAddr;				//Write start address
	endaddr=WriteAddr+NumToWrite*4;	//Write end address
	if(addrx<0X1FFF0000)			//Only for main storage area, need to execute erase operation!!
	{
		while(addrx<endaddr)		//Scan entire sector (for non-FFFFFFFF locations, erase first)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//If there are non-0XFFFFFFFF locations, need to erase sector
			{
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V range!!
				if(status!=FLASH_COMPLETE)break;	//Erase operation failed
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//Write data
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//Write data
			{
				break;	//Write exception
			}
			WriteAddr+=4;
			pBuffer++;
		}
	}
    FLASH_DataCacheCmd(ENABLE);	//After FLASH erase complete, enable data cache
	FLASH_Lock();//Lock
} 

//Calculate pages of Flash needed (Size in bytes)
//u32 Flash_PagesMask(vu32 Size)
//{
//	u32 pagenumber = 0x0;
//	u32 size = Size;
//	
//	if ((size % PAGE_SIZE) != 0)
//	{
//		pagenumber = (size / PAGE_SIZE) + 1;
//	}
//	else
//	{
//		pagenumber = size / PAGE_SIZE;
//	}
//	return pagenumber;

//}

//Prepare flash space, directly erase 1 sector
s32 Flash_Prepared(u32 Address, u32 Len)
{
//	u32 NbrOfPage = 0;
//	s32 i;
	
	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	u32 FlashDestination = Address;
	
	FLASH_Unlock();

    FLASHStatus = FLASH_EraseSector(STMFLASH_GetFlashSector(FlashDestination),VoltageRange_3);
    if(FLASHStatus != FLASH_COMPLETE)
        return 0;
    
	FLASH_Lock();
	
	return 1;
}

s32 IAP_EraseSWFlash(u32 Address, u32 size)
{
	s32 result = 0x00;

	__disable_irq();   //Disable all interrupts
	if(!Flash_Prepared(Address, size))
		result = 0x01;
	__enable_irq();		//Enable all interrupts

	return result;
}

//WriteAddr: start address (this address must be multiple of 4!!)
//pBuffer: data pointer
//NumToWrite: word (32-bit) count (number of 32-bit data to write)
void STMFLASH_Write_Word(u32 WriteAddr,s32 *pBuffer,u32 NumToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
	u32 addrx = 0;
	u32 endaddr = 0;	
    if (WriteAddr < STM32_FLASH_BASE|| WriteAddr % 4)
	{
		SetErrorByte(CTRLER_ERROR_E2PROM_BREAK);
        return;	//Illegal address
	}
	FLASH_Unlock();									//Unlock
    FLASH_DataCacheCmd(DISABLE);    //During FLASH erase, must disable data cache
 		
	addrx = WriteAddr;				//Write start address
	endaddr = WriteAddr + NumToWrite * 4;	//Write end address
	if (addrx < 0x1FFF0000)			//Only for main storage area, need to execute erase operation!!
	{
		while (addrx < endaddr)		//Scan entire sector (for non-FFFFFFFF locations, erase first)
		{
			if (STMFLASH_ReadWord(addrx)!= 0xFFFFFFFF)//If there are non-0XFFFFFFFF locations, need to erase sector
			{
				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V range!!
				if(status != FLASH_COMPLETE)
				{
					SetErrorByte(CTRLER_ERROR_E2PROM_BREAK);
                    break;	            //Erase operation failed
				}
			}
			else 
			{
				addrx += 4;
			}
		} 
	}
	if (status == FLASH_COMPLETE)
	{
		while (WriteAddr < endaddr)//Write data
		{
			if (FLASH_ProgramWord(WriteAddr,*pBuffer) != FLASH_COMPLETE)//Write data
			{
				SetErrorByte(CTRLER_ERROR_E2PROM_BREAK);
				break;	//Write exception
			}
			WriteAddr += 4;
			pBuffer ++;
			ClearErrorByte(CTRLER_ERROR_E2PROM_BREAK);
		}
	}
    FLASH_DataCacheCmd(ENABLE);	//After FLASH erase complete, enable data cache
	FLASH_Lock();//Lock
} 
//WriteAddr: start address (this address must be multiple of 4!!)
//pBuffer: data pointer
//NumToWrite: word (32-bit) count (number of 32-bit data to write)
void STMFLASH_Write_byteword(u32 WriteAddr,u8 *pBuffer,u8 NumToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
//	u32 addrx=0;
	u32 endaddr=0;	
    if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4) return;	//Illegal address

    FLASH_Unlock();									//Unlock
    FLASH_DataCacheCmd(DISABLE);//During FLASH erase, must disable data cache

//	addrx = WriteAddr;				//Write start address
	endaddr = WriteAddr + NumToWrite;	//Write end address
//	if(addrx < 0X1FFF0000)			//Only for main storage area, need to execute erase operation!!
//	{
//		while (addrx < endaddr)		//Scan entire sector (for non-FFFFFFFF locations, erase first)
//		{
//			if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)//If there are non-0XFFFFFFFF locations, need to erase sector
//			{
//				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V range!!
//				if (status!=FLASH_COMPLETE) break;	//Erase operation failed
//			} 
//            else 
//                addrx += 4;
//		} 
//	}
	if(status == FLASH_COMPLETE)
	{
		while (WriteAddr < endaddr)//Write data
		{
			if (FLASH_ProgramByte(WriteAddr, *pBuffer) != FLASH_COMPLETE)//Write data
			{
				break;	//Write exception
			}
			WriteAddr += 1;
			pBuffer ++;
		}
	}
    FLASH_DataCacheCmd(ENABLE);	//After FLASH erase complete, enable data cache
	FLASH_Lock();//Lock
} 
//Read specified length of data starting from specified address
//ReadAddr: start address
//pBuffer: data pointer
//NumToRead: word (4-byte) count
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//Read 4 bytes
		ReadAddr+=4;//Offset 4 bytes	
	}
}
//Read specified length of data starting from specified address
//ReadAddr: start address
//pBuffer: data pointer
//NumToRead: word (4-byte) count
void STMFLASH_Read_Word(u32 ReadAddr,s32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i = 0; i < NumToRead; i ++)
	{
		pBuffer[i] = STMFLASH_ReadWord(ReadAddr);//Read 4 bytes
		ReadAddr += 4;                          //Offset 4 bytes	
	}
}
/*******************************************************************************
* Function Name : UpdateCmdArrayFunc
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void UpdateCmdArrayFunc(void)//Restore to default parameters
{
	u16 i = 0;
    for (i = 0; i < HL_CMDMAP_LEN; i ++)
    {
        g_HLCmdMap[i] = 0;
    }   
    
    g_HLCmdMap[0] = 0x5A5A5A5A;

    /*Enable*/
//    g_HLCmdMap[HL_ENA_PRS_LOOP] = 0;
//    g_HLCmdMap[HL_ENA_PWR_LOOP] = 0;
    g_HLCmdMap[HL_ENA_LEAKEAGE] = 0;
//    g_HLCmdMap[HL_ENA_PARA_SAVE] = 0;
//    g_HLCmdMap[HL_ENA_PUMP_STRT] = 0;

    /**/
    g_HLCmdMap[HL_PRD_ANG_LOOP] = User_Parameter.TimPeriod->g_u16_Ang_PI_LOOP_Period ;
    g_HLCmdMap[HL_PRD_PRS_LOOP] = User_Parameter.TimPeriod->g_u16_Prs_PI_LOOP_Period;
    g_HLCmdMap[HL_PRD_ANG_ERR_D] = User_Parameter.TimPeriod->g_u16_Angle_Err_D_Period;
    g_HLCmdMap[HL_PRD_PRS_ERR_D] = User_Parameter.TimPeriod->g_u16_Prs_Err_D_Period;
    g_HLCmdMap[HL_PRD_ANG_FDB_D] = User_Parameter.TimPeriod->g_u16_Ang_Fdb_D_Period;
    g_HLCmdMap[HL_PRD_PRS_FDB_D] = User_Parameter.TimPeriod->g_u16_Prs_Fdb_D_Period;

    /*Note: These are floating point numbers, pay attention to precision*/
    g_HLCmdMap[HL_TIM_C_ANG_REF_FILTER] = (s32)(User_Parameter.LowPassFilterTimConst->g_f32_Ang_Ref_FilterTimPara * 10000);
    g_HLCmdMap[HL_TIM_C_ANG_FDB_FILTER] = (s32)(User_Parameter.LowPassFilterTimConst->g_f32_Ang_Fdb_FilterTimPara * 10000);
    g_HLCmdMap[HL_TIM_C_PRS_FDB_FILTER] = (s32)(User_Parameter.LowPassFilterTimConst->g_f32_Prs_Fdb_FilterTimPara * 10000);

    /**/
    g_HLCmdMap[HL_SWING_AMP] = (s32)(User_Parameter.AngSwingPara->g_f32_AngSwinging_Amp * 10000);
    g_HLCmdMap[HL_SWING_FREQ] = User_Parameter.AngSwingPara->g_u16_AngSwinging_Freq;

    /**/
    g_HLCmdMap[HL_CTRL_ANG_P_COMP_RATIO] = User_Parameter.g_u16_AngleLoop_P_Offset;
    g_HLCmdMap[HL_CTRL_PRS_COMP_ANG_RATIO] = (s32)(User_Parameter.g_f32_PrsCompAngRatio * 10000);
    g_HLCmdMap[HL_CTRL_LKGE_COMP_RATIO]= (s32)(User_Parameter.g_f32_leakage_compensation * 10000);
    
    /*PWM parameters*/
    g_HLCmdMap[HL_PI_ANG_P] = Angle_Loop.Kp;
    g_HLCmdMap[HL_PI_ANG_I] = Angle_Loop.Ki;
    g_HLCmdMap[HL_PI_ANG_ERR_D] = Angle_Loop.Kd;
    g_HLCmdMap[HL_PI_ANG_D] = Angle_Loop.Kv;
    g_HLCmdMap[HL_PI_ANG_P_DIV] = (s32)Angle_Loop.Kp_div;
    g_HLCmdMap[HL_PI_ANG_I_DIV] = (s32)Angle_Loop.Ki_div;
    g_HLCmdMap[HL_PI_ANG_ERR_D_DIV] = (s32)Angle_Loop.Kd_div;
    g_HLCmdMap[HL_PI_ANG_D_DIV] = Angle_Loop.Kv_div;
    g_HLCmdMap[HL_PI_ANG_LOWER_OUTPUT_LMT] = Angle_Loop.output_min;
    g_HLCmdMap[HL_PI_ANG_UPPER_OUTPUT_LMT] = Angle_Loop.output_max;
    g_HLCmdMap[HL_PI_ANG_I_AREA] = Angle_Loop.PI_area;
    g_HLCmdMap[HL_PI_ANG_I_AREA_L] = Angle_Loop.PI_area_l;
    g_HLCmdMap[HL_PI_ANG_AREA_I] = Angle_Loop.AREA_I;
    g_HLCmdMap[HL_PI_ANG_ERR_PERIOD] = Angle_Loop.Err_D_Period;
    
    g_HLCmdMap[HL_PI_PRS_P] = Pressure_Loop.Kp;
    g_HLCmdMap[HL_PI_PRS_I] = Pressure_Loop.Ki;
    g_HLCmdMap[HL_PI_PRS_ERR_D] = Pressure_Loop.Kd;
    g_HLCmdMap[HL_PI_PRS_D] = Pressure_Loop.Kv;
    g_HLCmdMap[HL_PI_PRS_P_DIV] = (s32)Pressure_Loop.Kp_div;
    g_HLCmdMap[HL_PI_PRS_I_DIV] = (s32)Pressure_Loop.Ki_div;
    g_HLCmdMap[HL_PI_PRS_ERR_D_DIV] = (s32)Pressure_Loop.Kd_div;
    g_HLCmdMap[HL_PI_PRS_D_DIV] = Pressure_Loop.Kv_div;
    g_HLCmdMap[HL_PI_PRS_LOWER_OUTPUT_LMT] = Pressure_Loop.output_min;
    g_HLCmdMap[HL_PI_PRS_UPPER_OUTPUT_LMT] = Pressure_Loop.output_max;
    g_HLCmdMap[HL_PI_PRS_I_AREA] = Pressure_Loop.PI_area;
    g_HLCmdMap[HL_PI_PRS_I_AREA_L] = Pressure_Loop.PI_area_l;
    g_HLCmdMap[HL_PI_PRS_AREA_I] = Pressure_Loop.AREA_I;
    g_HLCmdMap[HL_PI_PRS_ERR_PERIOD] = Pressure_Loop.Err_D_Period;
    /**/
    g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MIN] = User_Parameter.SysPara->u16AngAdc_MIN;
	g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MID]	= User_Parameter.SysPara->u16AngAdc_MID;
	g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MID_SCOPE] = User_Parameter.SysPara->u16AngAdc_MID_Scope;
    g_HLCmdMap[HL_SYS_PARA_ANG_ADC_MAX] = User_Parameter.SysPara->u16AngAdc_MAX;
    g_HLCmdMap[HL_SYS_PARA_ANG_ADC_SCOPE] = User_Parameter.SysPara->u16AngPerUnitScope;
    g_HLCmdMap[HL_SYS_PARA_PRS_ADC_MIN] = User_Parameter.SysPara->u16PrsAdc_MIN;
    g_HLCmdMap[HL_SYS_PARA_PRS_ADC_MAX] = User_Parameter.SysPara->u16PrsAdc_MAX;
    g_HLCmdMap[HL_SYS_PARA_CC_MIN] = User_Parameter.SysPara->u16PumpCC_MIN;
    g_HLCmdMap[HL_SYS_PARA_CC_MAX] = User_Parameter.SysPara->u16PumpCC_MAX;
    g_HLCmdMap[HL_SYS_PARA_DAC_OUT_MIN] = User_Parameter.SysPara->u16Valve_4mA_Dac;
    g_HLCmdMap[HL_SYS_PARA_DAC_OUT_MAX] = User_Parameter.SysPara->u16Valve_20mA_Dac;
    g_HLCmdMap[HL_SYS_PARA_DAC_OUT_MID] = User_Parameter.SysPara->u16Valve_MidPosi_Dac;
}



void Save_Parameter_Process(void)
{
    if (User_Parameter.PpqEna->PumpStrt_Ena == 0)
    {
        if (User_Parameter.PpqEna->ParaSaveEna == 1)
        {
            UpdateCmdArrayFunc();
            STMFLASH_Write_Word(STM32_FLASH_EEPROM_BASE, g_HLCmdMap, HL_CMDMAP_LEN);
            User_Parameter.PpqEna->ParaSaveEna = 0;
        }
    }
    else
    {
        User_Parameter.PpqEna->ParaSaveEna = 0;
    }
  
}

/*******************************************************************************
* Function Name : InitUserParaProcess
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void InitUserParaProcess(void)
{
	u16 i = 0;
    
    for (i = 0; i < HL_CMDMAP_LEN; i ++)
    {
        g_HLCmdMap[i] = 0;
    }   
    
	STMFLASH_Read_Word(STM32_FLASH_EEPROM_BASE, g_HLCmdMap, HL_CMDMAP_LEN);
    
    #if (1)
	if(g_HLCmdMap[0] == 0x5A5A5A5A)
	{
        /*Enable*/
//        User_Parameter.PpqEna->PrsLoop_Ena = g_HLCmdMap[HL_ENA_PRS_LOOP];
//        User_Parameter.PpqEna->PwrLoop_Ena = g_HLCmdMap[HL_ENA_PWR_LOOP];
        User_Parameter.PpqEna->AngLeak_Ena = g_HLCmdMap[HL_ENA_LEAKEAGE];
//        g_HLCmdMap[HL_ENA_PARA_SAVE] = 0;
//        User_Parameter.PpqEna->ParaSaveEna = g_HLCmdMap[HL_ENA_PARA_SAVE];
//        g_HLCmdMap[HL_ENA_PUMP_STRT] = 0;
//        User_Parameter.PpqEna->PumpStrt_Ena = g_HLCmdMap[HL_ENA_PUMP_STRT];
        
        /**/
        User_Parameter.TimPeriod->g_u16_Ang_PI_LOOP_Period = g_HLCmdMap[HL_PRD_ANG_LOOP];
        User_Parameter.TimPeriod->g_u16_Prs_PI_LOOP_Period = g_HLCmdMap[HL_PRD_PRS_LOOP];
        User_Parameter.TimPeriod->g_u16_Angle_Err_D_Period = g_HLCmdMap[HL_PRD_ANG_ERR_D];
        User_Parameter.TimPeriod->g_u16_Prs_Err_D_Period = g_HLCmdMap[HL_PRD_PRS_ERR_D];
        User_Parameter.TimPeriod->g_u16_Ang_Fdb_D_Period = g_HLCmdMap[HL_PRD_ANG_FDB_D];
        User_Parameter.TimPeriod->g_u16_Prs_Fdb_D_Period = g_HLCmdMap[HL_PRD_PRS_FDB_D];
        
        /*Note: These are floating point numbers, pay attention to precision*/
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
	else
	{
        User_Parameter.PpqEna->PrsLoop_Ena = 0;
        User_Parameter.PpqEna->PwrLoop_Ena = 0;
        User_Parameter.PpqEna->AngLeak_Ena = 0;       
        User_Parameter.PpqEna->ParaSaveEna = 0;
        User_Parameter.PpqEna->PumpStrt_Ena = 0;
        
        /**/
        User_Parameter.TimPeriod->g_u16_Ang_PI_LOOP_Period = ANG_PI_LOOP_CYCLE;
        User_Parameter.TimPeriod->g_u16_Prs_PI_LOOP_Period = PRS_PI_LOOP_CYCLE;
        User_Parameter.TimPeriod->g_u16_Angle_Err_D_Period = ANG_ERR_D_CYCLE;
        User_Parameter.TimPeriod->g_u16_Prs_Err_D_Period = PRS_ERR_D_CYCLE;
        User_Parameter.TimPeriod->g_u16_Ang_Fdb_D_Period = ANG_D_FILTER_CNT;
        User_Parameter.TimPeriod->g_u16_Prs_Fdb_D_Period = PRS_D_FILTER_CNT;
        
        /*Note: These are floating point numbers, pay attention to precision*/
        User_Parameter.LowPassFilterTimConst->g_f32_Ang_Ref_FilterTimPara = ANG_REF_FILTER_TIM_CONST;
        User_Parameter.LowPassFilterTimConst->g_f32_Ang_Fdb_FilterTimPara = ANG_FDB_FILTER_TIM_CONST;
        User_Parameter.LowPassFilterTimConst->g_f32_Prs_Fdb_FilterTimPara = PRS_FDB_FILTER_TIM_CONST;
        
        /**/
        User_Parameter.AngSwingPara->g_f32_AngSwinging_Amp  = ANGSHAKEAMP;
        User_Parameter.AngSwingPara->g_u16_AngSwinging_Freq = ANGSHAKEFREQ;
        
        /**/
        User_Parameter.g_u16_AngleLoop_P_Offset = 100;
        User_Parameter.g_f32_PrsCompAngRatio = 0.0f;
        User_Parameter.g_f32_leakage_compensation = 0.0005f;
        
        /**/
        User_Parameter.SysPara->u16AngAdc_MIN = ANG_FDB_MIN_ADC_VAL;
        User_Parameter.SysPara->u16AngAdc_MAX = ANG_FDB_MAX_ADC_VAL;
		User_Parameter.SysPara->u16AngAdc_MID = ANG_FDB_MID_ADC_VAL;
		User_Parameter.SysPara->u16AngAdc_MID_Scope = ANG_FDB_MID_ADC_VAL_SCOPE;
        User_Parameter.SysPara->u16AngPerUnitScope = ANG_PERUNIT_SCOPE;
        User_Parameter.SysPara->u16PrsAdc_MIN = PRS_FDB_MIN_ADC_VAL;
        User_Parameter.SysPara->u16PrsAdc_MAX = PRS_FDB_MAX_ADC_VAL;
        User_Parameter.SysPara->u16PumpCC_MIN = PUMP_MIN_CC_VAL;
        User_Parameter.SysPara->u16PumpCC_MAX = PUMP_MAX_CC_VAL;
        User_Parameter.SysPara->u16Valve_4mA_Dac = VALVE_4MA_VAL;
        User_Parameter.SysPara->u16Valve_20mA_Dac = VALVE_20MA_VAL;
        User_Parameter.SysPara->u16Valve_MidPosi_Dac = VALVE_MID_POSI_VAL;
	}
    #else
        User_Parameter.PpqEna->PrsLoop_Ena = 0;
        User_Parameter.PpqEna->PwrLoop_Ena = 0;
        User_Parameter.PpqEna->AngLeak_Ena = 0;       
        User_Parameter.PpqEna->ParaSaveEna = 0;
        User_Parameter.PpqEna->PumpStrt_Ena = 0;
        
        /**/
        User_Parameter.TimPeriod->g_u16_Ang_PI_LOOP_Period = ANG_PI_LOOP_CYCLE;
        User_Parameter.TimPeriod->g_u16_Prs_PI_LOOP_Period = PRS_PI_LOOP_CYCLE;
        User_Parameter.TimPeriod->g_u16_Angle_Err_D_Period = ANG_ERR_D_CYCLE;
        User_Parameter.TimPeriod->g_u16_Prs_Err_D_Period = PRS_ERR_D_CYCLE;
        User_Parameter.TimPeriod->g_u16_Ang_Fdb_D_Period = ANG_D_FILTER_CNT;
        User_Parameter.TimPeriod->g_u16_Prs_Fdb_D_Period = PRS_D_FILTER_CNT;
        
        /*Note: These are floating point numbers, pay attention to precision*/
        User_Parameter.LowPassFilterTimConst->g_f32_Ang_Ref_FilterTimPara = ANG_REF_FILTER_TIM_CONST;
        User_Parameter.LowPassFilterTimConst->g_f32_Ang_Fdb_FilterTimPara = ANG_FDB_FILTER_TIM_CONST;
        User_Parameter.LowPassFilterTimConst->g_f32_Prs_Fdb_FilterTimPara = PRS_FDB_FILTER_TIM_CONST;
        
        /**/
        User_Parameter.AngSwingPara->g_f32_AngSwinging_Amp  = ANGSHAKEAMP;
        User_Parameter.AngSwingPara->g_u16_AngSwinging_Freq = ANGSHAKEFREQ;
        
        /**/
        User_Parameter.g_u16_AngleLoop_P_Offset = 100;
        User_Parameter.g_f32_PrsCompAngRatio = 0.0f;
        User_Parameter.g_f32_leakage_compensation = 0.0005f;
        
        for (i = 0; i < HL_CMDMAP_LEN; i ++)
        {
            g_HLCmdMap[i] = 0;
        }
        /**/
        User_Parameter.SysPara->u16AngAdc_MIN = ANG_FDB_MIN_ADC_VAL;
        User_Parameter.SysPara->u16AngAdc_MAX = ANG_FDB_MAX_ADC_VAL;
		User_Parameter.SysPara->u16AngAdc_MID = ANG_FDB_MID_ADC_VAL;
		User_Parameter.SysPara->u16AngAdc_MID_Scope = ANG_FDB_MID_ADC_VAL_SCOPE;
        User_Parameter.SysPara->u16AngPerUnitScope = ANG_PERUNIT_SCOPE;
        User_Parameter.SysPara->u16PrsAdc_MIN = PRS_FDB_MIN_ADC_VAL;
        User_Parameter.SysPara->u16PrsAdc_MAX = PRS_FDB_MAX_ADC_VAL;
        User_Parameter.SysPara->u16PumpCC_MIN = PUMP_MIN_CC_VAL;
        User_Parameter.SysPara->u16PumpCC_MAX = PUMP_MAX_CC_VAL;
        User_Parameter.SysPara->u16Valve_4mA_Dac = VALVE_4MA_VAL;
        User_Parameter.SysPara->u16Valve_20mA_Dac = VALVE_20MA_VAL;
        User_Parameter.SysPara->u16Valve_MidPosi_Dac = VALVE_MID_POSI_VAL;
    #endif
    
     /*Update PWM parameters*/
    Update_PWM_PidPara(&Pwm_Output_A, &Pwm_Output_B, &Angle_Loop, &Pressure_Loop);
}

FLASH_Status ota_flash_erase(u32 addrx)//Application program area 0x0804 0000, bootloader area 0x0802 0000
{
	FLASH_Status status = FLASH_COMPLETE;
	FLASH_Unlock();									//Unlock
	FLASH_DataCacheCmd(DISABLE);//During FLASH erase, must disable data cache
	status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V range!!
	FLASH_DataCacheCmd(ENABLE);	//After FLASH erase complete, enable data cache
	FLASH_Lock();//Lock
	return status;
}

FLASH_Status ota_write_byteword(u32 WriteAddr,u8 *pBuffer,u32 NumToWrite)//Write one Byte to flash
{
	u32 endaddr=0;
	FLASH_Status status = FLASH_COMPLETE;
	FLASH_Unlock();									//Unlock
	FLASH_DataCacheCmd(DISABLE);//During FLASH erase, must disable data cache
	endaddr=WriteAddr+NumToWrite;
	while(WriteAddr<endaddr)//Write data
	{
		status = FLASH_ProgramByte(WriteAddr,*pBuffer);
		if(status!=FLASH_COMPLETE)//Write data
		{
			break;	//Write exception
		}
		WriteAddr+=1;
		pBuffer++;
	}

	FLASH_DataCacheCmd(ENABLE);	//After FLASH erase complete, enable data cache
	FLASH_Lock();//Lock
	return status;
}
/*******************************************************************************
* Function Name : UpdateSetCmdFunc()
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void UpdateSetCmdFunc()
{
    User_Parameter.PpqEna->PumpStrt_Ena = g_PCSetCmd[HL_ENA_PUMP_STRT];
    User_Parameter.PpqEna->PrsLoop_Ena =  g_PCSetCmd[HL_ENA_PRS_LOOP];
    User_Parameter.PpqEna->PwrLoop_Ena =  g_PCSetCmd[HL_ENA_PWR_LOOP];
	User_Parameter.PpqEna->ParaSaveEna =  g_PCSetCmd[HL_ENA_PARA_SAVE];
	
    if (User_Parameter.PpqEna->PumpStrt_Ena == 0)
        g_u8_ExchangeFlg = g_PCSetCmd[HL_CMD_OPN_LOOP];
    else
        g_u8_ExchangeFlg = 0;
    
    InstructionSet.TiltAngRef = g_PCSetCmd[HL_CTRL_USB_ANG_REF] * 100;
    InstructionSet.g_u16_PerUnitPrsVal = g_PCSetCmd[HL_CTRL_USB_PRS_REF] * 100;
    InstructionSet.g_f32_TorqueLimt = (float)g_PCSetCmd[HL_CTRL_USB_PWR_REF];
}
