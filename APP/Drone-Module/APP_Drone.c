#include "APP_Drone.h"

// THR_Status thr_status     = THR_FREE;   // 默认空闲
Drone_Status drone_status = Drone_LOCK; // 默认锁定

// typedef enum {
//     THR_MAX,
//     THR_FREE,
//     THR_LEAVE_MAX,
//     THR_MIN,
// } THR_Status;

// uint32_t enter_THR_FREE_tick      = 0;
// uint32_t enter_THR_MAX_tick       = 0;
// uint32_t enter_THR_MIN_tick       = 0;
// uint32_t enter_THR_LEAVE_MAX_tick = 0;

// void APP_Flight_Unlock(void)
// {
//     if (rc_status == RC_CONNECTED) {
//         if (flight_status == Flight_LOCK) {
//             switch (thr_status) {
//                 case THR_FREE:
//                     if (rc_data.THR >= 950) {
//                         thr_status         = THR_MAX;
//                         enter_THR_MAX_tick = xTaskGetTickCount();
//                     }
//                     break;
//                 case THR_MAX:
//                     if (rc_data.THR < 950) {
//                         if (xTaskGetTickCount() - enter_THR_MAX_tick >= 1000) {
//                             thr_status               = THR_LEAVE_MAX;
//                             enter_THR_LEAVE_MAX_tick = xTaskGetTickCount();
//                         } else {
//                             thr_status = THR_FREE;
//                         }
//                     }
//                     break;
//                 case THR_LEAVE_MAX:
//                     // 当油门状态处于THR_LEAVE_MAX状态超过2000ms，返回THR_FREE状态
//                     if (xTaskGetTickCount() - enter_THR_LEAVE_MAX_tick > 2000) {
//                         thr_status = THR_FREE;
//                     }
//                     if (rc_data.THR <= 50) {
//                         thr_status         = THR_MIN;
//                         enter_THR_MIN_tick = xTaskGetTickCount();
//                     }
//                     break;
//                 case THR_MIN:
//                     // if (rc_data.THR > 10) {
//                     if (xTaskGetTickCount() - enter_THR_MIN_tick >= 1000) {
//                         thr_status = THR_FREE;
//                         // 更新飞机状态为 解锁
//                         flight_status = Flight_UNLOCK;
//                         // } else {
//                         //     thr_status = THR_FREE;
//                         // }
//                     }
//                     break;
//             }
//         }
//     } else {
//         flight_status = Flight_LOCK;
//         thr_status    = THR_FREE;
//     }
//     printf("thr_status: %d, flight_status: %d\n", thr_status, flight_status);
// }

void APP_Drone_Update(void)
{
    // 更新RC数据到电机速度

    if (drone_status == Drone_LOCK) {
        // 如果无人机处于锁定状态，保持油门为0
        leftTopMotor.speed =
            leftBottomMotor.speed =
                rightTopMotor.speed =
                    rightBotomMotor.speed = 0;
    } else if (drone_status == Drone_NORMAL) {
        leftTopMotor.speed =
            leftBottomMotor.speed =
                rightTopMotor.speed =
                    rightBotomMotor.speed = rc_data.THR;
    } else if (drone_status == Drone_HOLD_HIGH) {
        /* ********************************* */
        leftTopMotor.speed =
            leftBottomMotor.speed =
                rightTopMotor.speed =
                    rightBotomMotor.speed = 50;
    } else if (drone_status == Drone_FAULT) {
        // 故障状态
        leftTopMotor.speed =
            leftBottomMotor.speed =
                rightTopMotor.speed =
                    rightBotomMotor.speed = 0;
    }
    // 更新电机速度
    // Int_Motor_AllWork();
}

static uint32_t UnlockFlight_tick = 0;
void APP_Drone_Unlock(void)
{
    // 检查解锁条件：解锁标志为1且油门值为0
    if (rc_data.isUnlockFlight == 1 && rc_data.THR == 0) {
        if (UnlockFlight_tick == 0) {
            // 第一次满足条件，记录当前时间
            UnlockFlight_tick = xTaskGetTickCount();
        } else {
            // 检查是否持续了1.5秒（1500ms）
            if (xTaskGetTickCount() - UnlockFlight_tick >= 1500) {
                drone_status      = Drone_NORMAL;
                UnlockFlight_tick = 0; // 重置计时器
            }
        }
    } else {
        // 条件不满足，重置计时器
        UnlockFlight_tick = 0;
    }
}

void APP_Drone_Status_Update(void)
{
    switch (drone_status) {
        case Drone_LOCK:
            if (rc_status == RC_CONNECTED) {
                APP_Drone_Unlock();
            }
            break;
        case Drone_NORMAL:
            if (rc_status == RC_UNCONNECTED) {
                drone_status = Drone_FAULT;
            } else {
                if (rc_data.isFixHeightPoint == 1) {
                    drone_status = Drone_HOLD_HIGH;
                }
            }
            break;
        case Drone_HOLD_HIGH:
            if (rc_status == RC_UNCONNECTED) {
                drone_status = Drone_FAULT;
            } else {
                if (rc_data.isFixHeightPoint == 0) {
                    drone_status = Drone_NORMAL;
                }
            }
            break;
        case Drone_FAULT:
            if (rc_status == RC_CONNECTED) {
                drone_status = Drone_NORMAL;
            }
            break;
    }
}

void APP_Drone_Start(void)
{
    APP_Drone_Status_Update();
    APP_Drone_Update();
}
