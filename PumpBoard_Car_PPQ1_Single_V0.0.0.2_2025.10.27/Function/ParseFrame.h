#ifndef PARASEFRAME_H
#define PARASEFRAME_H

#include "malloc.h"
#include "stm32f4xx.h"

/*CMD相关*/
#define CMD_READ                        1
#define CMD_WRITE                       2
#define CMD_DRAW_WAVE                   3
#define CMD_WR_ACK                      4
#define PARA_WRITE                      5
#define PARA_READ                       6
#define TX_FRMBUF_SIZE				    128			//发送队列每帧缓存大小


//帧相关宏定义
#define FRM_CMD			                2
#define FRM_INDEX		                3
#define FRM_LEN			                4
#define FRM_DATA		                5


void USB_PackDataArray(uint8_t *p,uint32_t *p1);
//void USB_PackWRBackDataArray(uint8_t *p);
void USB_UploadDataPC(uint8_t *p,uint8_t *p1);
void USB_ParseData(u8 *buf, u32);
uint16_t Crc16(uint8_t *ptr, uint32_t);
u8 USB_TransDataFormat(u8, u8, u8, u8* pdata, u8* FormatBuf);

#endif
