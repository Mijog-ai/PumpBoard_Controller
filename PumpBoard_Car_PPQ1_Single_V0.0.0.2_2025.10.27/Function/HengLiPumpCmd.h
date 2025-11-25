#ifndef HL_CMD_H
#define HL_CMD_H

#define HL_RDCMD_LEN                0x30
#define HL_WRCMD_LEN                0x30
//指令类型宏定义
#define HL_CMDTYPE_RD				0x01		//读指令
#define HL_CMDTYPE_WR				0x02		//写指令
#define HL_CMDTYPE_WR_NR			0x03		//写控制表指令（无返回）

//内存控制表宏定义
#define HL_CMDMAP_LEN				256			//内存控制表总长度（int16单位）
#define HL_CMDMAP_INDLEN			16 			//内存控制表主索引数
#define HL_CMDMAP_SUBLEN			16 			//内存控制表子索引数

//读索引
#define	HL_INF_SW_VERS				0x01        //软件版本

#define HL_INF_ANG_ADC_FDB          0x02        //角度ADC采样
#define	HL_INF_PRS_ADC_FDB			0x03        //压力ADC采样
#define	HL_INF_ANALOG_OUTPUT		0x04		//模拟输出
#define HL_INF_ANG_PER_UNIT         0x05		//角度反馈标幺值(0-10000)
#define HL_INF_PRS_PER_UNIT         0x06		//压力反馈标幺值(0-10000)
#define	HL_FDB_USB_ANG_REF			0x07		//角度设定值0-10000
#define	HL_FDB_USB_PRS_REF			0x08		//压力设定值0-10000
#define HL_INF_PRS_VAL_BAR          0x09		//压力反馈实际值
#define HL_INF_CUR_A_FDB            0x0A		//电磁阀A电流反馈
#define HL_INF_CUR_B_FDB            0x0B		//电磁阀B电流反馈

#define HL_INF_CC_MIN               0x0C		//最小排量
#define HL_INF_CC_MAX               0x0D		//最大排量
#define HL_INF_PI_ANG_P             0x0E		//PID角度Kp
#define HL_INF_PI_ANG_P_DIV         0x0F		//PID角度Kp_div
#define HL_INF_PI_ANG_ERR_D         0x10		//PID角度Kd
#define HL_INF_PI_ANG_ERR_D_DIV     0x11		//PID角度Kd_div
#define HL_INF_PI_PRS_P             0x12		//PID压力Kp
#define HL_INF_PI_PRS_P_DIV         0x13	//PID压力Kp_div
#define HL_INF_PI_PRS_ERR_D         0x14		//PID压力Kd
#define HL_INF_PI_PRS_ERR_D_DIV     0x15		//PID压力Kd_div

#define HL_INF_ANG_MIN_ADC_FDB      0x16		//角度ADC最小值
#define HL_INF_ANG_MID_ADC_FDB      0x17		//角度ADC中间值
#define HL_INF_ANG_MID_SCOPE_FDB    0x18		//角度ADC中间值对应角度设定百分比
#define HL_INF_ANG_MAX_ADC_FDB      0x19		//角度ADC最大值
#define HL_INF_PRS_MIN_ADC_FDB      0x1A		//压力ADC最小值
#define HL_INF_PRS_MAX_ADC_FDB      0x1B		//压力ADC最大值

#define	HL_ERROR_READY				0x1C		//无故障
#define HL_ERROR_CurFdb_A_BREAK     0x1D		//电磁阀A故障
#define HL_ERROR_CurFdb_B_BREAK     0x1E		//电磁阀B故障
#define HL_ERROR_ANGSENSOR_BREAK    0x1F		//摆角传感器故障
#define HL_ERROR_PRSSENSOR_01_BREAK 0x20		//压力传感器1故障
#define HL_ERROR_PRSSENSOR_02_BREAK 0x21		//压力传感器2故障
#define HL_ERROR_CHKE_BREAK         0x22		//校验故障
#define HL_ERROR_E2PROM_BREAK       0x23		//参数保存失败
#define HL_ERROR_BUS_BREAK          0x24		//总线故障

//控制指令 写 索引
#define	HL_ENA_PUMP_STRT			0x01
#define	HL_ENA_PRS_LOOP				0x02
#define	HL_ENA_PWR_LOOP				0x03
#define	HL_ENA_PARA_SAVE			0x04

#define HL_CMD_OPN_LOOP             0x05

#define HL_CTRL_USB_ANG_REF         0x06
#define HL_CTRL_USB_PRS_REF         0x07
#define HL_CTRL_USB_PWR_REF         0x08

//控制参数 (写) 索引
#define	HL_ENA_LEAKEAGE				0x01

#define	HL_PRD_ANG_LOOP				0x08
#define	HL_PRD_PRS_LOOP				0x09
#define	HL_PRD_ANG_ERR_D			0x0A
#define	HL_PRD_PRS_ERR_D			0x0B
#define	HL_PRD_ANG_FDB_D			0x0C
#define	HL_PRD_PRS_FDB_D			0x0D

#define	HL_TIM_C_ANG_REF_FILTER		0x0E
#define	HL_TIM_C_ANG_FDB_FILTER		0x0F
#define	HL_TIM_C_PRS_FDB_FILTER		0x10

#define	HL_SWING_FREQ				0x11
#define	HL_SWING_AMP				0x12

#define	HL_CTRL_ANG_P_COMP_RATIO	0x13
#define	HL_CTRL_PRS_COMP_ANG_RATIO	0x14
#define	HL_CTRL_LKGE_COMP_RATIO		0x15

#define	HL_PI_ANG_P					0x16
#define	HL_PI_ANG_P_DIV				0x17
#define	HL_PI_ANG_I					0x18
#define	HL_PI_ANG_I_DIV				0x19
#define	HL_PI_ANG_ERR_D				0x1A
#define	HL_PI_ANG_ERR_D_DIV			0x1B
#define	HL_PI_ANG_D					0x1C
#define	HL_PI_ANG_D_DIV				0x1D
#define	HL_PI_ANG_LOWER_OUTPUT_LMT	0x1E
#define	HL_PI_ANG_UPPER_OUTPUT_LMT	0x1F
#define	HL_PI_PRS_P					0x20
#define	HL_PI_PRS_P_DIV				0x21
#define	HL_PI_PRS_I					0x22
#define	HL_PI_PRS_I_DIV				0x23
#define	HL_PI_PRS_ERR_D				0x24
#define	HL_PI_PRS_ERR_D_DIV			0x25
#define	HL_PI_PRS_D					0x26
#define	HL_PI_PRS_D_DIV				0x27
#define	HL_PI_PRS_LOWER_OUTPUT_LMT	0x28
#define	HL_PI_PRS_UPPER_OUTPUT_LMT	0x29


#define HL_SYS_PARA_ANG_ADC_MIN     0x2A
#define HL_SYS_PARA_ANG_ADC_MID     0x2B
#define HL_SYS_PARA_ANG_ADC_MID_SCOPE   0x2C
#define HL_SYS_PARA_ANG_ADC_MAX     0x2D
#define HL_SYS_PARA_ANG_ADC_SCOPE   0x2E
#define HL_SYS_PARA_PRS_ADC_MIN     0x2F
#define HL_SYS_PARA_PRS_ADC_MAX     0x30
#define HL_SYS_PARA_CC_MIN          0x31
#define HL_SYS_PARA_CC_MAX          0x32
#define HL_SYS_PARA_DAC_OUT_MIN     0x33
#define HL_SYS_PARA_DAC_OUT_MAX     0x34
#define HL_SYS_PARA_DAC_OUT_MID     0x35

#define HL_PI_ANG_I_AREA            0x36
#define HL_PI_ANG_I_AREA_L          0x37
#define HL_PI_PRS_I_AREA            0x38
#define HL_PI_PRS_I_AREA_L          0x39
#define HL_PI_ANG_AREA_I            0x3A
#define HL_PI_PRS_AREA_I            0x3B
#define HL_PI_ANG_ERR_PERIOD        0x3C
#define HL_PI_PRS_ERR_PERIOD        0x3D
#endif

