#include "limit.h"
/** 
 * @brief 限幅函数
 * @param x 限幅变量
 * @param l_bound 下限
 * @param h_bound 上限
 * @return 无
 * @detailed 有容错性，如果上下限传递的顺序不正确，仍然可以正确限幅
 */
void Limit_Q31(int* x, int l_bound, int h_bound)
{
	if(h_bound > l_bound)
	{
		if(*x > h_bound)
			(*x) = h_bound;
		else if(*x < l_bound)
			(*x) = l_bound;
	}
	else if(h_bound < l_bound)
	{
		if(*x < h_bound)
			(*x) = h_bound;
		else if(*x > l_bound)
			(*x) = l_bound;
	}
	else
		(*x) = l_bound;
}

void Limit_f32(float* x, float l_bound, float h_bound)
{
	if(h_bound > l_bound)
	{
		if(*x > h_bound)
			(*x) = h_bound;
		else if(*x < l_bound)
			(*x) = l_bound;
	}
	else if(h_bound < l_bound)
	{
		if(*x < h_bound)
			(*x) = h_bound;
		else if(*x > l_bound)
			(*x) = l_bound;
	}
	else
		(*x) = l_bound;
}

/** 
 * @brief 限幅函数
 * @param x 限幅变量
 * @param l_bound 下限
 * @param h_bound 上限
 * @return 无
 */
void Limit_Q15(short* x, short l_bound, short h_bound)
{
	if(h_bound > l_bound)
	{
		if(*x > h_bound)
			(*x) = h_bound;
		else if(*x < l_bound)
			(*x) = l_bound;
	}
	else if(h_bound < l_bound)
	{
		if(*x < h_bound)
			(*x) = h_bound;
		else if(*x > l_bound)
			(*x) = l_bound;
	}
	else
		(*x) = l_bound;
}

/** 
 * @brief 速率受限的跟踪
 * @param x 跟踪变量
 * @param target 跟踪目标
 * @return 无
 */
void Rate_Tracking_Q31 (volatile int* x, int target, int tracking_rate)
{
	if((*x) == target)
		return;
	if(tracking_rate <= 0)
	{
		if(tracking_rate == 0)
			(*x) = target;
		return;
	}
	
	if((*x) - target > tracking_rate)
		(*x) -= tracking_rate;
	else if((*x) - target < -tracking_rate)
		(*x) += tracking_rate;
	else
		(*x) = target;
}

void DeadZoneLimit_Q31(int* x, int DeadZone)
{
	if(DeadZone <= 0)
		return;
	(*x) = (*x) > DeadZone ? ((*x) - DeadZone) : ((*x) < -DeadZone ? ((*x) + DeadZone) : 0);
}
/*******************************************************************************
* Function Name : Rate_Tracking_Q15
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void Rate_Tracking_Q15(short* x,short target,short tracking_rate)
{
	if((*x) == target)
		return;
	if(tracking_rate <= 0)
	{
		if(tracking_rate == 0)
			(*x) = target;
		return;
	}
	
	if((*x) - target > tracking_rate)
		(*x) -= tracking_rate;
	else if((*x) - target < -tracking_rate)
		(*x) += tracking_rate;
	else
		(*x) = target;
}
