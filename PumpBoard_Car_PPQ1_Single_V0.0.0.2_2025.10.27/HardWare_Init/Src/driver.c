/*	
* 实现底层驱动的编写
* 
*/
#include "application.h" 



sys_t sys;










void Can1_Config(void)
{
		// 500k =6  250k = 12  125k = 24
	if(user_parameter.baud_rate_canopen==125)
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		
	}
	else if(user_parameter.baud_rate_canopen==250)
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,12,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		
	}
	else if(user_parameter.baud_rate_canopen==500)
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		
	}
	else//默认250k
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,12,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
	
	}
}
void Can1_Reset_Config(void)
{
		// 500k =6  250k = 12  125k = 24
	if(user_parameter.baud_rate_canopen==125)
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
	}
	else if(user_parameter.baud_rate_canopen==250)
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,12,CAN_Mode_Normal);
	}
	else if(user_parameter.baud_rate_canopen==500)
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);
	}
	else//默认250k
	{
		CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,12,CAN_Mode_Normal);
	}
}
void Can2_Config(void)
{
		// 500k =6  250k = 12  125k = 24
	if(user_parameter.baud_rate_j1939==125) 
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,24,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		
	}
	else if(user_parameter.baud_rate_j1939==250)
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,12,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);CAN_Mode_LoopBack
		
	}
	else if(user_parameter.baud_rate_j1939==500)
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,6,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		
	}
	else//默认250k
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,12,CAN_Mode_Normal);
		//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
		
	}
}
void Can2_Reset_Config(void)
{
		// 500k =6  250k = 12  125k = 24
	if(user_parameter.baud_rate_j1939==125) 
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,24,CAN_Mode_Normal);
	}
	else if(user_parameter.baud_rate_j1939==250)
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,12,CAN_Mode_Normal);
	}
	else if(user_parameter.baud_rate_j1939==500)
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,6,CAN_Mode_Normal);
	}
	else//默认250k
	{
		CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_11tq,12,CAN_Mode_Normal);
	}
}
void judder_basic_parameter_updata_process(void)
{
	static uint16_t period=0,duty=0,cnt=0;
	cnt++;
	if(cnt>=100)
	{
			cnt=0;//100ms 更新一次
			if(sys.judder_basic_duty>95)
			{
				sys.judder_basic_duty=95;
			}
			else if(sys.judder_basic_duty<85)
			{
				sys.judder_basic_duty=85;
			}
			if(sys.judder_basic_frequency<50)
			{
				sys.judder_basic_frequency=50;
			}
			else if(sys.judder_basic_frequency>200)
			{
				sys.judder_basic_frequency=200;
			}
			if(period != sys.judder_basic_frequency || duty != sys.judder_basic_duty)
			{
				period = sys.judder_basic_frequency;
				duty = sys.judder_basic_duty;
				TIM_SetAutoreload(TIM5,(15000-period*50));
				TIM_SetCompare1(TIM5,((15000-period*50)*duty/100.0));
			}
	}
}
