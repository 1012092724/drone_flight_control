#include "APP_LED.h"

/*
    遥控状态：
    未连接：
        左上灯灭
        右上灯灭
    已连接：
        左上灯亮
        右上灯亮
飞机自身状态：
    空闲状态：
        后两个灯隔1秒闪烁（慢闪烁）
    正常状态：
        后两个灯等隔100毫秒闪烁（快闪烁）
    定高状态：
        后两个灯常亮
    失败状态
        后两个灯关闭
*/

void App_LED_Start(void)
{
    static uint16_t last_toggle_time = 0; // 上一次切换LED的时间
    switch (rc_status) {
        case RC_CONNECTED:
            Int_LED_On(Left_Top_LED);
            Int_LED_On(Right_Top_LED);
            break;
        case RC_UNCONNECTED:
            Int_LED_Off(Left_Top_LED);
            Int_LED_Off(Right_Top_LED);
            break;
    }
    switch (drone_status) {
        case eDrone_IDLE:
            if (xTaskGetTickCount() - last_toggle_time >= 1000) {
                Int_LED_Toggle(Left_Bottom_LED);
                Int_LED_Toggle(Right_Bottom_LED);
                last_toggle_time = xTaskGetTickCount();
            }
            break;
        case eDrone_NORMAL:
            if (xTaskGetTickCount() - last_toggle_time >= 100) {
                Int_LED_Toggle(Left_Bottom_LED);
                Int_LED_Toggle(Right_Bottom_LED);
                last_toggle_time = xTaskGetTickCount();
            }
            break;
        case eDrone_HOLD_HIGH:
            Int_LED_On(Left_Bottom_LED);
            Int_LED_On(Right_Bottom_LED);
            break;
        case eDrone_FAULT:
            Int_LED_Off(Left_Bottom_LED);
            Int_LED_Off(Right_Bottom_LED);
            break;
    }
}