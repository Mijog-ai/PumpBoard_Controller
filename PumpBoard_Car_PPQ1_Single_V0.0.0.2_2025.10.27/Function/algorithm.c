#include "algorithm.h"

//uint16_t find_min_value(uint16_t x[],uint16_t n)
//{
//	int min_value=0,i=0;
//	min_value = x[0];
//	for(i=1;i<n;i++)
//	{
//		if(min_value>x[i])
//		{
//			min_value = x[i] ;
//		}
//	}
//	return min_value;
//}

//uint16_t find_max_value(uint16_t x[],uint16_t n)
//{
//	int max_value=0,i=0;
//	max_value = x[0];
//	for(i=1;i<n;i++)
//	{
//		if(max_value<x[i])
//		{
//			max_value = x[i] ;
//		}
//	}
//	return max_value;
//}

uint16_t reversal_0_100_value(uint16_t data)
{
	return (100-data);
}

uint16_t reversal_0_1000_value(uint16_t data)
{
	return (1000-data);
}

//输入模拟信号 最小值 最大值shu
//输出0--100的线性值
uint16_t input_signal_change_0_100value(uint16_t input_value,uint16_t signal_min,uint16_t signal_max)
{	
	uint16_t cur_value = input_value;//当前值
	uint16_t diff_value = signal_max - signal_min;
  if(cur_value<=signal_min)	
	{
		cur_value = signal_min;
	}
	else if(cur_value>=signal_max)
	{
		cur_value=signal_max;
	}
	//根据实际的信号转换成0--100的数值
  cur_value = cur_value * 100/diff_value - 100 * signal_min / diff_value;
	 
	return cur_value;
}

//输入模拟信号 最小值 最大值
//输出0--1000的线性值
uint16_t input_signal_change_0_1000value(uint16_t input_value,uint16_t signal_min,uint16_t signal_max)
{	
	uint16_t cur_value = input_value;//当前值
	uint16_t diff_value = signal_max - signal_min;
  if(cur_value<=signal_min)	
	{
		cur_value = signal_min;
	}
	else if(cur_value>=signal_max)
	{
		cur_value=signal_max;
	}
	//根据实际的信号转换成0--100的数值
  cur_value = cur_value * 1000/diff_value - 1000 * signal_min / diff_value;
	 
	return cur_value;
}

