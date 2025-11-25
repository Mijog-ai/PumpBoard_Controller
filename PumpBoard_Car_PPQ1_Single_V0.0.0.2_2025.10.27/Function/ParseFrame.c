#include "string.h"
#include "ParseFrame.h"
#include "IAP.h"
#include "flash.h"
#include "PID.h"
#include "HengLiPumpCmd.h"

void Update_CtrlPara(void);

typedef struct _USB_Frame {				//数据发送帧队列结构体
	s32 len;													//帧长度
	u8 buf[TX_FRMBUF_SIZE];			//帧数据
} USB_Frame;

u8 g_u8_UploadInfoArray[TX_FRMBUF_SIZE] = {0};
u8 g_u8_UploadIAPInfoArray[TX_FRMBUF_SIZE] = {0};
extern  s32 g_HLCmdMap[HL_CMDMAP_LEN];
extern  u32 g_PCRdCmd[HL_RDCMD_LEN];
extern  u16 g_PCSetCmd[HL_WRCMD_LEN];

/* CRC16 余式表 */
static uint16_t crctalbeabs[] = { 
	0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 
	0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400 
};

/*!
 *  功  能: CRC16校验
 *  param1: 指向要校验的数据的指针
 *  param2: 要校验的数据的长度
 *  retval: 校验所得到的值，uint16_t 类型
 * 
 *  说  明: 本次CRC校验为查表法，多项式为 x16+x15+x2+1(0x8005)，CRC的初始值为0xFFFF
 */
uint16_t Crc16(uint8_t *ptr, uint32_t len) 
{
	uint16_t crc = 0xffff; 
	uint32_t i;
	uint8_t ch;
 
	for (i = 0; i < len; i++) 
    {
		ch = *ptr++;
		crc = crctalbeabs[(ch ^ crc) & 15] ^ (crc >> 4);
		crc = crctalbeabs[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
	} 
	
	return crc;
}
/*******************************************************************************
* Function Name : ParseFrame
* Description   : USB解析数据
* Input         : 		
* Output        : 
* Return        : 
*******************************************************************************/
void ParseFrame(u8 *buf)
{
    u8 IAP_Result = 0;
//    u32 t_u32_IAPBinLength = 0;
    switch (buf[FRM_CMD])
    {
        case CMD_READ:
        case CMD_DRAW_WAVE:
            USB_PackDataArray(buf, g_PCRdCmd);
            USB_UploadDataPC(buf, g_u8_UploadInfoArray);
        break;
        
        case PARA_READ:
            buf[FRM_CMD] = PARA_READ;
            USB_PackDataArray(buf, (u32 *)g_HLCmdMap);//USB_PackWRBackDataArray(buf);
            USB_UploadDataPC(buf, g_u8_UploadInfoArray);
        break;
        
        case CMD_WRITE:             
            memcpy(&g_PCSetCmd[buf[FRM_INDEX]], &buf[FRM_DATA], buf[FRM_LEN]);
            UpdateSetCmdFunc();
            buf[FRM_CMD] = CMD_WR_ACK;
            USB_PackDataArray(buf, g_PCRdCmd);
            USB_UploadDataPC(buf, g_u8_UploadInfoArray);
        break;
        
        case PARA_WRITE:
            memcpy(&g_HLCmdMap[buf[FRM_INDEX]], &buf[FRM_DATA], buf[FRM_LEN]);
            Update_PWM_PidPara(&Pwm_Output_A, &Pwm_Output_B, &Angle_Loop, &Pressure_Loop);
            Update_CtrlPara();                                                  //更新除PID之外的参数
            buf[FRM_CMD] = PARA_READ;
            USB_PackDataArray(buf, (u32 *)g_HLCmdMap);//USB_PackWRBackDataArray(buf, (u16*)g_HLCmdMap);
            USB_UploadDataPC(buf, g_u8_UploadInfoArray);
        break;
        
        case IAP_BEGIN:
            memcpy(&m_IAP_FileSize, &buf[FRM_DATA], 4);	//固件大小
            if (IAP_EraseSWFlash(STM32_FLASH_BACKUP_BASE, m_IAP_FileSize + 128) == 0)
            {
                g_u8_UploadIAPInfoArray[0] = 0;
                buf[FRM_CMD] = IAP_ACK;
                USB_UploadDataPC(buf, g_u8_UploadIAPInfoArray);
            }           
        break;
        
        case IAP_WRITE:
            IAP_Result = IAP_FlashWriteFunc(buf);

            g_u8_UploadIAPInfoArray[0] = IAP_Result;
            buf[FRM_CMD] = IAP_ACK;
            USB_UploadDataPC(buf, g_u8_UploadIAPInfoArray);

        break;
            
        case IAP_END:
            PrepareSystemReset();
        
            g_u8_UploadIAPInfoArray[0] = 0;
            buf[FRM_CMD] = IAP_ACK;
            USB_UploadDataPC(buf, g_u8_UploadIAPInfoArray);
        break;
        
        case IAP_REST:
            NVIC_SystemReset();
        break;
        
        default:
		break;
    }
}
/*******************************************************************************
* Function Name : USB_ParseData
* Description   : USB解析数据
* Input         : 		
* Output        : 
* Return        : 
*******************************************************************************/
u16 Scope_s_rxPos = 0;
void USB_ParseData(u8 *buf, u32 ParseDataLen)
{
//	static unsigned char s_RXBuf[EUART2_RX_BUF_SIZE] = {0};
	static unsigned char s_bFrameHead1 = 0;
	static unsigned char s_bBeginFrame = 0;
	static u16 s_rxPos = 0;
//	static u16 s_rxLen = 0;
	u16 s_csum = 0;
	unsigned short temp16 = 0;
	
	if(s_bBeginFrame)   //是否开始新的一帧
	{			
        if (s_rxPos > ParseDataLen)
        {
            s_bFrameHead1 = s_bBeginFrame = 0;
            s_csum = s_rxPos = 0;
            if (buf[s_rxPos] == 0x5A && !s_bFrameHead1)                 //ZP     20240904   调试过程中遇到:无法进入IAP_END，因为(s_bBeginFrame = 1 && (s_rxPos > ParseDataLen))
            {
                s_rxPos = 0;
                s_bFrameHead1 = 1;
                s_rxPos ++;
            }
            return;
        }       
        else
        {            
            if(s_rxPos == ParseDataLen - 2)        //一帧数据传输完毕
            { 
                temp16 = buf[ParseDataLen - 2] + (buf[ParseDataLen - 1] << 8);
                s_csum = Crc16(&buf[2],ParseDataLen - 4);
                if(s_csum == temp16)
                    ParseFrame(buf);  //解析帧数据
                s_bFrameHead1 = s_bBeginFrame = 0;
                s_rxPos = 0;
                s_csum = 0;
                Scope_s_rxPos = s_rxPos;
                return;
            }
        }

        s_rxPos ++;
	}
	else if(buf[s_rxPos] == 0x5A && !s_bFrameHead1)
	{
		s_rxPos = 0;
        s_bFrameHead1 = 1;
        s_rxPos ++;
	}
	else if(buf[s_rxPos] == 0xA5 && s_bFrameHead1 && !s_bBeginFrame)
	{
		s_bBeginFrame = 1;
        s_rxPos ++;
		s_csum = 0;
	}
	else
	{
		s_bFrameHead1 = s_bBeginFrame = 0;
		s_csum = 0;
	}

    Scope_s_rxPos = s_rxPos;
}
u8 USB_TransDataFormat(u8 Cmd, u8 Index, u8 DataLen, u8* pdata, u8* FormatBuf)
{
    u8 pos = 0;
    u8 i = 0;
    u16 csum = 0;

    FormatBuf[pos++] = 0x5A;			
	FormatBuf[pos++] = 0xA5;						
	FormatBuf[pos++] = Cmd;
	FormatBuf[pos++] = Index;
	FormatBuf[pos++] = DataLen;
	for(i = 0; i < DataLen; i++)
	{
		FormatBuf[pos++] = pdata[i];
	}
    
    csum = Crc16(&FormatBuf[2], pos - 2);
	FormatBuf[pos++] = csum;
	FormatBuf[pos++] = csum >> 8;
    
    return pos;
}
/*******************************************************************************
* Function Name : USB_PackDataArray
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void USB_PackDataArray(u8 *Buf, u32 *p)
{
    u8 i = 0;
    u8 j = 0;
    u8 t_u8_MapIndex = Buf[FRM_INDEX];
    
//    memcpy(g_u8_UploadInfoArray,(u16*)&g_HLCmdMap[t_u8_MapIndex], Buf[FRM_LEN]);
//    
    for (i = 0; i < (Buf[FRM_LEN]); i++)
    {
        if (i == 0)
            g_u8_UploadInfoArray[i] = (u8)p[t_u8_MapIndex];
        else if (i == 1)
            g_u8_UploadInfoArray[i] = p[t_u8_MapIndex] >> 0x08;
        else
        {
            if (i % 2 == 0)
            {
                j++;
                g_u8_UploadInfoArray[i] = (u8)p[(t_u8_MapIndex + j)];      //低位
            }
            else
                g_u8_UploadInfoArray[i] = p[(t_u8_MapIndex + j)] >> 0x08;
        }
    }  
}
/*******************************************************************************
* Function Name : USB_PackWRBackDataArray
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
//void USB_PackWRBackDataArray(u8 *Buf)
//{
//    u8 i = 0;
//    u8 j = 0;
//    u8 t_u8_MapIndex = Buf[FRM_INDEX];
//    
////    memcpy(g_u8_UploadInfoArray,(u16*)&g_HLCmdMap[t_u8_MapIndex], Buf[FRM_LEN]);
////    
//    for (i = 0; i < (Buf[FRM_LEN]); i++)
//    {
//        if (i == 0)
//            g_u8_UploadInfoArray[i] = (u8)g_HLCmdMap[t_u8_MapIndex];
//        else if (i == 1)
//            g_u8_UploadInfoArray[i] = g_HLCmdMap[t_u8_MapIndex] >> 0x08;
//        else
//        {
//            if (i % 2 == 0)
//            {
//                j++;
//                g_u8_UploadInfoArray[i] = (u8)g_HLCmdMap[(t_u8_MapIndex + j)];      //低位
//            }
//            else
//                g_u8_UploadInfoArray[i] = g_HLCmdMap[(t_u8_MapIndex + j)] >> 0x08;
//        }
//    }  
//}

