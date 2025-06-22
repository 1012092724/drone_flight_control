#include "APP_LED.h"

void App_LED_Start(void)
{
    static uint32_t last_toggle_time = 0;
    uint32_t current_tick            = xTaskGetTickCount();

    /* 安全处理tick溢出情况 */
    uint32_t elapsed_time = current_tick - last_toggle_time;

    switch (rc_status) {
        case RC_CONNECTED: // 已连接状态
            switch (drone_status) {
                case Drone_LOCK: // 锁定状态 左下 右下灯 慢闪烁
                    if (elapsed_time >= pdMS_TO_TICKS(500)) {
                        Int_LED_On(Left_Top_LED);
                        Int_LED_On(Right_Top_LED);
                        Int_LED_Toggle(Left_Bottom_LED);
                        Int_LED_Toggle(Right_Bottom_LED);
                        last_toggle_time = current_tick;
                    }
                    break;
                case Drone_NORMAL: // 正常状态 四灯 常亮
                    Int_LED_On(Left_Top_LED);
                    Int_LED_On(Right_Top_LED);
                    Int_LED_On(Left_Bottom_LED);
                    Int_LED_On(Right_Bottom_LED);
                    break;
                case Drone_HOLD_HIGH: // 定高状态 四灯 快闪烁
                    if (elapsed_time >= pdMS_TO_TICKS(250)) {
                        Int_LED_Toggle(Left_Top_LED);
                        Int_LED_Toggle(Right_Top_LED);
                        Int_LED_Toggle(Left_Bottom_LED);
                        Int_LED_Toggle(Right_Bottom_LED);
                        last_toggle_time = current_tick;
                    }
                    break;
            }
            break;
        case RC_UNCONNECTED: // 未连接状态 四灯闪烁
            switch (drone_status) {
                case Drone_FAULT:
                    if (elapsed_time >= pdMS_TO_TICKS(50)) {
                        Int_LED_Toggle(Left_Top_LED);
                        Int_LED_Toggle(Right_Top_LED);
                        Int_LED_Toggle(Left_Bottom_LED);
                        Int_LED_Toggle(Right_Bottom_LED);
                        last_toggle_time = current_tick;
                    }
                    break;
                default:
                    // 同步状态
                    if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) != HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin)) {
                        // 全部关闭
                        Int_LED_Off(Left_Top_LED);
                        Int_LED_Off(Right_Top_LED);
                        Int_LED_Off(Left_Bottom_LED);
                        Int_LED_Off(Right_Bottom_LED);
                    }
                    if (elapsed_time >= pdMS_TO_TICKS(1000)) {
                        Int_LED_Toggle(Left_Top_LED);
                        Int_LED_Toggle(Right_Top_LED);
                        Int_LED_Toggle(Left_Bottom_LED);
                        Int_LED_Toggle(Right_Bottom_LED);
                        last_toggle_time = current_tick;
                    }
                    break;
            }
            break;
    }
}