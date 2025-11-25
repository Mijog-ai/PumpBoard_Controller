#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__
#include "stdint.h"

//寻找最小值
//uint16_t find_min_value(uint16_t x[],uint16_t n);
//寻找最大值
//uint16_t find_max_value(uint16_t x[],uint16_t n);
//0--100反转输出
uint16_t reversal_0_100_value(uint16_t data);
//0--1000反转输出
uint16_t reversal_0_1000_value(uint16_t data);
//输入信号 转成0--100输出
uint16_t input_signal_change_0_100value(uint16_t input_value,uint16_t signal_min,uint16_t signal_max);
//输入信号 转成0--1000输出
uint16_t input_signal_change_0_1000value(uint16_t input_value,uint16_t signal_min,uint16_t signal_max);
























#endif
