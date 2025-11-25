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

//读取指定地址的半字(16位数据) 
//faddr:读地址 
//返回值:对应数据.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
} 

//返回值:对应数据.
u16 STMFLASH_Read_HalfWord(u32 faddr)
{
	return *(vu32*)faddr; 
} 
//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
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
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
  FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
  if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
	FLASH_Unlock();									//解锁 
  FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx=WriteAddr;				//写入的起始地址
	endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
	if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx<endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status!=FLASH_COMPLETE)break;	//发生错误了
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//写数据
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
    FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 

//计算将会用到多少页Flash（Size字节为单位）
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

//准备flash空间，直接擦除1个扇区
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

	__disable_irq();   //关闭总中断
	if(!Flash_Prepared(Address, size))
		result = 0x01;
	__enable_irq();		//开放总中断

	return result;
}

//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write_Word(u32 WriteAddr,s32 *pBuffer,u32 NumToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
	u32 addrx = 0;
	u32 endaddr = 0;	
    if (WriteAddr < STM32_FLASH_BASE|| WriteAddr % 4)
	{
		SetErrorByte(CTRLER_ERROR_E2PROM_BREAK);
        return;	//非法地址
	}
	FLASH_Unlock();									//解锁 
    FLASH_DataCacheCmd(DISABLE);    //FLASH擦除期间,必须禁止数据缓存
 		
	addrx = WriteAddr;				//写入的起始地址
	endaddr = WriteAddr + NumToWrite * 4;	//写入的结束地址
	if (addrx < 0x1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while (addrx < endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if (STMFLASH_ReadWord(addrx)!= 0xFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status != FLASH_COMPLETE)
				{
					SetErrorByte(CTRLER_ERROR_E2PROM_BREAK);
                    break;	            //发生错误了
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
		while (WriteAddr < endaddr)//写数据
		{
			if (FLASH_ProgramWord(WriteAddr,*pBuffer) != FLASH_COMPLETE)//写入数据
			{ 
				SetErrorByte(CTRLER_ERROR_E2PROM_BREAK);
				break;	//写入异常
			}
			WriteAddr += 4;
			pBuffer ++;
			ClearErrorByte(CTRLER_ERROR_E2PROM_BREAK);
		} 
	}
    FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write_byteword(u32 WriteAddr,u8 *pBuffer,u8 NumToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
//	u32 addrx=0;
	u32 endaddr=0;	
    if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4) return;	//非法地址
    
    FLASH_Unlock();									//解锁 
    FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
//	addrx = WriteAddr;				//写入的起始地址
	endaddr = WriteAddr + NumToWrite;	//写入的结束地址
//	if(addrx < 0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
//	{
//		while (addrx < endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
//		{
//			if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
//			{   
//				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
//				if (status!=FLASH_COMPLETE) break;	//发生错误了
//			} 
//            else 
//                addrx += 4;
//		} 
//	}
	if(status == FLASH_COMPLETE)
	{
		while (WriteAddr < endaddr)//写数据
		{
			if (FLASH_ProgramByte(WriteAddr, *pBuffer) != FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr += 1;
			pBuffer ++;
		} 
	}
    FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
} 
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
	}
}
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read_Word(u32 ReadAddr,s32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i = 0; i < NumToRead; i ++)
	{
		pBuffer[i] = STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr += 4;                          //偏移4个字节.	
	}
}
/*******************************************************************************
* Function Name : UpdateCmdArrayFunc
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void UpdateCmdArrayFunc(void)//恢复出厂默认设置
{
	u16 i = 0;
    for (i = 0; i < HL_CMDMAP_LEN; i ++)
    {
        g_HLCmdMap[i] = 0;
    }   
    
    g_HLCmdMap[0] = 0x5A5A5A5A;
    
    /*使能*/
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

    /*☆   这是浮点数，注意精度*/
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
    
    /*PWM参数*/
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
        /*使能*/
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
        
        /*☆   这是浮点数，注意精度*/
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
        
        /*☆   这是浮点数，注意精度*/
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
        
        /*☆   这是浮点数，注意精度*/
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
    
     /*更新PWM参数*/
    Update_PWM_PidPara(&Pwm_Output_A, &Pwm_Output_B, &Angle_Loop, &Pressure_Loop);
}

FLASH_Status ota_flash_erase(u32 addrx)//备份程序放在0x0804 0000  主程序放在0x0802 0000
{
	FLASH_Status status = FLASH_COMPLETE;
	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
	status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V之间!!
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
	return status;
}

FLASH_Status ota_write_byteword(u32 WriteAddr,u8 *pBuffer,u32 NumToWrite)//写一个Byte到flash
{
	u32 endaddr=0;	
	FLASH_Status status = FLASH_COMPLETE;
	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
	endaddr=WriteAddr+NumToWrite;
	while(WriteAddr<endaddr)//写数据
	{
		status = FLASH_ProgramByte(WriteAddr,*pBuffer);
		if(status!=FLASH_COMPLETE)//写入数据
		{ 
			break;	//写入异常
		}
		WriteAddr+=1;
		pBuffer++;
	} 
		
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁
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
