#ifndef __INT_MOTOR_H__
#define __INT_MOTOR_H__

#include "tim.h"
#include "Com_Config.h"

extern Motor_Struct leftTopMotor;
extern Motor_Struct leftBottomMotor;
extern Motor_Struct rightTopMotor;
extern Motor_Struct rightBotomMotor;

void Int_Motor_Init(void);
void Int_Motor_AllWork(void);
void Int_Motor_UpdateSpeed(Motor_Struct *motor);

#endif /* __INT_MOTOR_H__ */