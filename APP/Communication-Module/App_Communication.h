#ifndef __APP_COMMUNICATION_H__
#define __APP_COMMUNICATION_H__

#include "Com_Debug.h"
#include "Int_Si24R1.h"
#include "Com_Config.h"

void App_Communication_Start(void);
Com_Status App_Communication_ReadRemoteData(void);
Com_Status App_Communication_ConnectCheck(Com_Status isReadData);

#endif /* __APP_COMMUNICATION_H__ */