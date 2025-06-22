#include "APP_Power.h"

uint32_t power_tick = 0;
void APP_Power_Control(void)
{
    if (rc_data.isPowerDonw) {
        // 如果遥控器数据中 isPowerDonw 为 1，则关闭电源
        Int_IP5305T_Close();
    } else {
        if (power_tick == 0) {
            power_tick = xTaskGetTickCount();
        } else if (xTaskGetTickCount() - power_tick > 20000) {
            // 20S 开一次电源
            Int_IP5305T_Open();
            power_tick = 0;
        }
    }
}