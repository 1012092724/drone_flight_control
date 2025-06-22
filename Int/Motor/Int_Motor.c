#include "Int_Motor.h"

Motor_Struct leftTopMotor    = {LEFT_TOP, 0};
Motor_Struct leftBottomMotor = {LEFT_BOTTOM, 0};
Motor_Struct rightTopMotor   = {RIGHT_TOP, 0};
Motor_Struct rightBotomMotor = {RIGHT_BOTTOM};

void Int_Motor_Init(void)
{

    //debug_printfln("Motor Init");
    /* left-top: TIM3_CH1 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    /* left-bottom: TIM4_CH4*/
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    /* right-top: TIM2_CH2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    /* right-bottom: TIM1_CH3 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 4个马达速度均初始化为0 */
    //debug_printfln("Motor Speed Set 0");

    leftTopMotor.speed =
        leftBottomMotor.speed =
            rightTopMotor.speed =
                rightBotomMotor.speed = 0;

    Int_Motor_AllWork();

    //debug_printfln("Motor Init Success!");
}

void Int_Motor_AllWork(void)
{
    Int_Motor_UpdateSpeed(&leftTopMotor);
    Int_Motor_UpdateSpeed(&leftBottomMotor);
    Int_Motor_UpdateSpeed(&rightTopMotor);
    Int_Motor_UpdateSpeed(&rightBotomMotor);
}

void Int_Motor_UpdateSpeed(Motor_Struct *motor)
{
    // motor->speed = rc_data.THR;

    motor->speed = motor->speed >= 1000 ? 1000 : motor->speed;

    if (motor->location == LEFT_TOP) {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, motor->speed);
    } else if (motor->location == LEFT_BOTTOM) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, motor->speed);
    } else if (motor->location == RIGHT_TOP) {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, motor->speed);
    } else if (motor->location == RIGHT_BOTTOM) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, motor->speed);
    }
}