#include "limit.h"
/**
 * @brief Limit function
 * @param x Variable to limit
 * @param l_bound Lower bound
 * @param h_bound Upper bound
 * @return None
 * @detailed Has fault tolerance, if upper and lower bounds are passed in wrong order, can still limit correctly
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
 * @brief Limit function
 * @param x Variable to limit
 * @param l_bound Lower bound
 * @param h_bound Upper bound
 * @return None
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
 * @brief Rate-limited tracking
 * @param x Tracking variable
 * @param target Tracking target
 * @return None
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
