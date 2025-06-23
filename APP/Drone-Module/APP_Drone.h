#ifndef __APP_DRONE_H__
#define __APP_DRONE_H__

#include "Com_Config.h"
#include "Int_Motor.h"
#include "Int_MPU6050.h"

void APP_Drone_Start(void);

void APP_Drone_Init(void);

void APP_Drone_Status_Update(void);

void APP_Drone_Unlock(void);

#endif /* __APP_DRONE_H__ */