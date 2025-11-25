#ifndef _CHECKTABLE_H
#define _CHECKTABLE_H

#include "stm32f4xx.h"

/******************************************* variable define ********************************************/
#define TABLE_UP_ROW                                21
#define TABLE_UP_COL                                2

//extern const u16 Table_Cur_Angle_Up[TABLE_UP_ROW][TABLE_UP_COL];
//extern const u16 Table_Cur_Angle_Down[TABLE_UP_ROW][TABLE_UP_COL];

/******************************************* function define ********************************************/
extern u16 CheckList_UP_Func(u16,const u16 (*p)[TABLE_UP_COL]);
extern u16 CheckList_Down_Func(u16, const u16 (*p)[TABLE_UP_COL]);
#endif/* _CHECKTABLE_H */
/*************  COPYRIGHT (C) 2019 ******************* END OF FILE **************************************/
