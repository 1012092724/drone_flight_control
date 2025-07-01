#ifndef __COM_PID_H__
#define __COM_PID_H__

#include "Com_Config.h"

void Com_PID_Cumpute(PID_Struct *pid);

void Com_PID_Casecade(PID_Struct *outer, PID_Struct *inner);

void Com_PID_Reset(PID_Struct *pid);

#endif /* __COM_PID_H__ */
