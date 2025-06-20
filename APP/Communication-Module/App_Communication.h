#ifndef __APP_COMMUNICATION_H__
#define __APP_COMMUNICATION_H__

#include "Com_Debug.h"
#include "Int_Si24R1.h"
#include "Com_Config.h"

void App_Communication_Start(void);
Data_Status App_Communication_ValidatePacket(void);
void App_Communication_UpdateConnectionStatus(Data_Status isReadData);

#endif /* __APP_COMMUNICATION_H__ */