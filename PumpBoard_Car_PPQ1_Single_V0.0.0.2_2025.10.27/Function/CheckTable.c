#include "CheckTable.h"

u16 CheckList_UP_Func(u16, const u16 (*p)[TABLE_UP_COL]);
u16 CheckList_Down_Func(u16, const u16 (*p)[TABLE_UP_COL]);
                                   
/*******************************************************************************
* Function Name : CheckList_UP_Func
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u16 CheckList_UP_Func(u16 ADCVal,const u16 (*TableArray)[TABLE_UP_COL])
{
//  u32 t_GasADVal = 0;
	u16 low,mid,high;
    u16 x1,x2,y1,y2,temp;
    low = 0;
    high = TABLE_UP_ROW - 1;              //
    mid = (low+high)>>1;
	
//	  t_GasADVal = (u32)(AngRef) * 16;
    if(ADCVal >= *(TableArray[high] + 0))//Table_Cur_Angle_Up[high][0])
    {
        return (*(TableArray[high] + 1));
    }
    if(ADCVal < *(TableArray[low] + 0))
    {
        return (*(TableArray[low] + 1));
    }
    do
    {
        if(ADCVal < *(TableArray[mid] + 0))
        {
            high = mid;
            mid = (low+high)>>1;
        }
        else if(ADCVal > *(TableArray[mid] + 0))
        {
            low = mid;
            mid = (low+high)>>1;
        }
        else
        {
            return (*(TableArray[mid] + 1));
        }
    }while(mid != (high - 1));
    if(ADCVal < *(TableArray[mid] + 0))
    {
        x1 = *(TableArray[mid-1] + 0);
        x2 = *(TableArray[mid] + 0);
        y1 = *(TableArray[mid-1] + 1);
        y2 = *(TableArray[mid] + 1);
    }
    else if(ADCVal > *(TableArray[mid] + 0))
    {
        x1 = *(TableArray[mid] + 0);
        x2 = *(TableArray[mid+1] + 0);
        y1 = *(TableArray[mid] + 1);
        y2 = *(TableArray[mid+1] + 1);
    }
    else
    {
        return (*(TableArray[mid] + 1));
    }
    temp = ADCVal;
    temp = y1 + (temp - x1)*(y2-y1)/(x2-x1);	
    return temp;
}
/*******************************************************************************
* Function Name : CheckList_Down_Func
* Description   : 
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
u16 CheckList_Down_Func(u16 ADCVal,const u16 (*TableArray)[TABLE_UP_COL])
{
//  u32 t_GasADVal = 0;
	u16 low,mid,high;
    u16 x1,x2,y1,y2,temp;
    low = 0;                     //这里是3的原因:后几组数据摆角并未再增加
    high = TABLE_UP_ROW - 1;             
    mid = (low+high)>>1;
	
//	  t_GasADVal = (u32)(AngRef) * 16;
    if(ADCVal <= *(TableArray[high] + 0))
    {
        return (*(TableArray[high] + 1));
    }
    if(ADCVal > *(TableArray[low] + 0))
    {
        return (*(TableArray[low] + 1));
    }
    do
    {
        if(ADCVal > *(TableArray[mid] + 0))
        {
            high = mid;
            mid = (low+high)>>1;
        }
        else if(ADCVal < *(TableArray[mid] + 0))
        {
            low = mid;
            mid = (low+high)>>1;
        }
        else
        {
            return (*(TableArray[mid] + 1));
        }
    }while(mid != (high - 1));
    if(ADCVal > *(TableArray[mid] + 0))
    {
        x1 = *(TableArray[mid-1] + 0);
        x2 = *(TableArray[mid] + 0);
        y1 = *(TableArray[mid-1] + 1);
        y2 = *(TableArray[mid] + 1);
    }
    else if(ADCVal < *(TableArray[mid] + 0))
    {
        x1 = *(TableArray[mid] + 0);
        x2 = *(TableArray[mid+1] + 0);
        y1 = *(TableArray[mid] + 1);
        y2 = *(TableArray[mid+1] + 1);
    }
    else
    {
        return (*(TableArray[mid] + 1));
    }
    temp = ADCVal;
    temp = y1 + (temp - x1)*(y2-y1)/(x2-x1);	
    return temp;
}
