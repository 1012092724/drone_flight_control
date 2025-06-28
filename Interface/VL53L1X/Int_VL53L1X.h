#ifndef __INT_VL53L1X_H__
#define __INT_VL53L1X_H__

#include "main.h"
#include "VL53L1X_api.h"

void Int_VL53L1X_Init(void);

void Int_VL53L1X_Get_Distance(uint16_t *distance);

#endif /* __INT_VL53L1X_H__ */