#ifndef __APP_DRONE_H__
#define __APP_DRONE_H__

#include "Com_Config.h"
#include "Int_Motor.h"
#include "Int_MPU6050.h"
#include "Com_Filter.h"
#include "Com_IMU.h"
#include "Com_PID.h"
#include "Int_VL53L1X.h"

void APP_Drone_Start(float run_cycle);

void APP_Drone_Init(void);

void printGyroAccel(GyroAccel_Struct *gyroAccel);

void printfEulerAngle(EulerAngle_Struct *eulerAngle);

// PID Debug

extern PID_Struct pitchPid;
extern PID_Struct gyroYPid;

extern PID_Struct rollPid;
extern PID_Struct gyroXPid;

extern PID_Struct yawPid;
extern PID_Struct gyroZPid;

#endif /* __APP_DRONE_H__ */