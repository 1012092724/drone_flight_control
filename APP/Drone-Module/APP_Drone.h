#ifndef __APP_DRONE_H__
#define __APP_DRONE_H__

#include "Com_Config.h"
#include "Int_Motor.h"
#include "Int_MPU6050.h"
#include "Com_Filter.h"
#include "Com_IMU.h"
#include "Com_PID.h"

void APP_Drone_Start(float run_cycle);

void APP_Drone_Init(void);

void printGyroAccel(GyroAccel_Struct *gyroAccel);
void printfEulerAngle(EulerAngle_Struct *eulerAngle);

#endif /* __APP_DRONE_H__ */