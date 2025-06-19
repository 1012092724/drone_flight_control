#ifndef __COM_TYPES_H__
#define __COM_TYPES_H__

typedef enum{
    eRC_UNCONNECTED,
    eRC_CONNECTED
}RC_Status_e;

typedef enum{
    eDrone_IDLE,
    eDrone_NORMAL,
    eDrone_HOLD_HIGH,
    eDrone_FAULT
}Drone_Status_e;

#endif /* __COM_TYPES_H__ */