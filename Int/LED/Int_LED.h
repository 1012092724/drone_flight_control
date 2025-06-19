#ifndef __INT_LED_H__
#define __INT_LED_H__
#include "main.h"

typedef enum {
    Left_Top_LED,
    Left_Bottom_LED,
    Right_Top_LED,
    Right_Bottom_LED
} LED_Position_e;

// typedef struct
// {
//     GPIO_TypeDef *GPIOx;
//     uint16_t GPIO_Pin;
// } LED_Handle_t;

// extern LED_Handle_t left_top_led;
// extern LED_Handle_t left_bottom_led;
// extern LED_Handle_t right_top_led;
// extern LED_Handle_t right_bottom_led;

// void Int_LED_On(LED_Handle_t *led_handle);
// void Int_LED_Off(LED_Handle_t *led_handle);

void Int_LED_On(LED_Position_e led_position);
void Int_LED_Off(LED_Position_e led_position);
void Int_LED_Toggle(LED_Position_e led_position);

#endif /* __INT_LED_H__ */
