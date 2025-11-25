#ifndef DEADZONE_CURRENT_H
#define DEADZONE_CURRENT_H

#include <stdint.h>
#include <math.h>
#include <limits.h>

#endif
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

int16_t Compute_Cur_B_Ref(
    int32_t s_s32_OffsetCur,
    int32_t DeadZone,
    uint16_t t_u16_SolDeadCur_B,
    uint16_t t_u16_SolDeadCur_B_Fall_B,
    uint16_t t_u16_Pump_Sol_B_Max_Cur_Ref,
    uint16_t t_u16_Pump_Sol_B_Min_Cur_Ref
);

