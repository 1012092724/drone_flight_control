#include "Int_LED.h"

// LED_Handle_t left_top_led = {
//     .GPIOx    = LED1_GPIO_Port,
//     .GPIO_Pin = LED1_Pin,
// };
// LED_Handle_t left_bottom_led = {
//     .GPIOx    = LED4_GPIO_Port,
//     .GPIO_Pin = LED4_Pin,
// };
// LED_Handle_t right_top_led = {
//     .GPIOx    = LED2_GPIO_Port,
//     .GPIO_Pin = LED2_Pin,
// };
// LED_Handle_t right_bottom_led = {
//     .GPIOx    = LED3_GPIO_Port,
//     .GPIO_Pin = LED3_Pin,
// };

// void Int_LED_On(LED_Handle_t *led_handle)
// {
//     HAL_GPIO_WritePin(led_handle->GPIOx, led_handle->GPIO_Pin, GPIO_PIN_SET);
// }
// void Int_LED_Off(LED_Handle_t *led_handle)
// {
//     HAL_GPIO_WritePin(led_handle->GPIOx, led_handle->GPIO_Pin, GPIO_PIN_RESET);
// }

/**
 * @brief  Turn on the specified LED
 * @param  led_position: LED position to turn on
 * @note   LED_ON corresponds to GPIO_PIN_RESET based on hardware design
 */
void Int_LED_On(LED_Position_e led_position)
{
    switch (led_position) {
        case Left_Top_LED:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            break;
        case Right_Top_LED:
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
            break;
        case Right_Bottom_LED:
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
            break;
        case Left_Bottom_LED:
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
            break;
    }
}

/**
 * @brief  Turn off the specified LED
 * @param  led_position: LED position to turn off
 * @note   LED_OFF corresponds to GPIO_PIN_SET based on hardware design
 */
void Int_LED_Off(LED_Position_e led_position)
{
    switch (led_position) {
        case Left_Top_LED:
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
            break;
        case Right_Top_LED:
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
            break;
        case Right_Bottom_LED:
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
            break;
        case Left_Bottom_LED:
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
            break;
    }
}

void Int_LED_Toggle(LED_Position_e led_position)
{
    switch (led_position) {
        case Left_Top_LED:
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            break;
        case Right_Top_LED:
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            break;
        case Right_Bottom_LED:
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            break;
        case Left_Bottom_LED:
            HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            break;
    }
}
