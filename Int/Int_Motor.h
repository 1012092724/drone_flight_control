#ifndef __INT_MOTOR_H__
#define __INT_MOTOR_H__

#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
    uint16_t speed;
} Motor_Handle_t;

extern Motor_Handle_t Left_Top_Motor;
extern Motor_Handle_t Left_Bottom_Motor;
extern Motor_Handle_t Right_Top_Motor;
extern Motor_Handle_t Right_Bottom_Motor;

void Int_Motor_Init(void);

void Int_Motor_UpdateSpeed(Motor_Handle_t *motor_handle);

#endif /* __INT_MOTOR_H__ */