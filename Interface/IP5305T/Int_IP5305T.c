#include "Int_IP5305T.h"

/**
 * @description: 打开IP5305T芯片
 *  IP5305T 可识别长按键和短按键操作，不需要按键时 PIN5 脚悬空。
 *
 *  key对应的引脚为PB2,应该设置为通用开漏模式: 当设置为高电平时相当于引脚悬空 设置为低电平时相当于按下按键
 *  短按唤醒时间为至少80ms, 这个地方手册有误. 我们设置为100ms
 * @return {*}
 */
void Int_IP5305T_Open(void)
{
    /* 短按 */
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
}

/**
 * @description: 关闭IP5305T芯片
 *  在1s内连续两次短按
 * @return {*}
 */
void Int_IP5305T_Close(void)
{
    /* 1 次短按 */
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);

    vTaskDelay(500);

    /* 2 次短按 */
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(100);
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
}
