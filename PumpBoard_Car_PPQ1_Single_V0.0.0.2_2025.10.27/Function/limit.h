#ifndef __LIMIT_H_
#define __LIMIT_H_

#define LIMIT(a, l, h)	(a > h ? h : (a < l ? l : a))

void Limit_Q31(int* x, int l_bound, int h_bound);
void Limit_f32(float* x, float l_bound, float h_bound);
void Limit_Q15(short* x, short l_bound, short h_bound);
void Rate_Limit_Tracking_Q31(int* x, int target, 
							int rate_limit_up,  int rate_limit_down);
void Rate_Limit_Tracking_Q15(short* x, short target, 
							short rate_limit_up,  short rate_limit_down);
void Rate_Tracking_Q31(volatile int* x, int target, int tracking_rate);
void DeadZoneLimit_Q31(int* x, int DeadZone);
void Rate_Tracking_Q15(short* x, short target, short tracking_rate);
#endif
