#include "Int_Motor.h"

Motor_Handle_t Left_Top_Motor     = {.Channel = TIM_CHANNEL_1, .htim = &htim3};
Motor_Handle_t Left_Bottom_Motor  = {.Channel = TIM_CHANNEL_4, .htim = &htim4};
Motor_Handle_t Right_Top_Motor    = {.Channel = TIM_CHANNEL_2, .htim = &htim2};
Motor_Handle_t Right_Bottom_Motor = {.Channel = TIM_CHANNEL_3, .htim = &htim1};

void Int_Motor_Init(void)
{
    HAL_TIM_PWM_Start(Left_Top_Motor.htim, Left_Top_Motor.Channel);
    HAL_TIM_PWM_Start(Left_Bottom_Motor.htim, Left_Bottom_Motor.Channel);
    HAL_TIM_PWM_Start(Right_Top_Motor.htim, Right_Top_Motor.Channel);
    HAL_TIM_PWM_Start(Right_Bottom_Motor.htim, Right_Bottom_Motor.Channel);
}

void Int_Motor_UpdateSpeed(Motor_Handle_t *motor_handle)
{
    // speed的取值范围为0-9999
    if (motor_handle->speed > 9999) {
        motor_handle->speed = 9999;
    }
    __HAL_TIM_SET_COMPARE(motor_handle->htim, motor_handle->Channel, motor_handle->speed);
}