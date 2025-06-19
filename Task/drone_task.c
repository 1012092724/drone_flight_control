#include "drone_task.h"
#include "Com_Types.h"
#include "Int_LED.h"

/* 全局变量区 begin */
RC_Status_e RC_Status       = eRC_UNCONNECTED; // 遥控器状态
Drone_Status_e Drone_Status = eDrone_IDLE;     // 无人机状态
/* 全局变量区 end */

// POWER任务
void Power_Task(void *pvParameters);
#define POWER_TASK_NAME       "Power_Task"
#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY   4
TaskHandle_t Power_Task_Handle;

// LED任务
void LED_Task(void *pvParameters);
#define LED_TASK_NAME       "LED_Task"
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY   1
TaskHandle_t LED_Task_Handle;

// 电机任务
void Motor_Task(void *pvParameters);
#define MOTOR_TASK_NAME       "Motor_Task"
#define MOTOR_TASK_STACK_SIZE 128
#define MOTOR_TASK_PRIORITY   2
TaskHandle_t Motor_Task_Handle;

void Sart_ALL_Task()
{
    // 电机初始化
    Int_Motor_Init();
    // 创建Power任务
    xTaskCreate(Power_Task, POWER_TASK_NAME, POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &Power_Task_Handle);
    // =================== LED TEST ======================= //
    // RC_Status    = eRC_CONNECTED;
    // Drone_Status = eDrone_NORMAL;
    // 创建LED任务
    xTaskCreate(LED_Task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_Task_Handle);
    // 创建Motor任务
    // ================== Motor TEST ======================= //
    Left_Bottom_Motor.speed  = 100;
    Right_Bottom_Motor.speed = 100;
    Left_Top_Motor.speed     = 100;
    Right_Top_Motor.speed    = 100;
    xTaskCreate(Motor_Task, MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, &Motor_Task_Handle);
    vTaskStartScheduler(); // Start the FreeRTOS scheduler
}

void Power_Task(void *pvParameters)
{
    while (1) {
        vTaskDelay(15000);
        HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
        vTaskDelay(100);
        HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
    }
}

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
void LED_Task(void *pvParameters)
{
    uint16_t last_toggle_time = 0; // 上一次切换LED的时间
    while (1) {
        // debug_printfln("LED_Task Running");
        switch (RC_Status) {
            case eRC_CONNECTED:
                Int_LED_On(Left_Top_LED);
                Int_LED_On(Right_Top_LED);
                break;
            case eRC_UNCONNECTED:
                Int_LED_Off(Left_Top_LED);
                Int_LED_Off(Right_Top_LED);
                break;
        }
        switch (Drone_Status) {
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
        vTaskDelay(100);
    }
}

void Motor_Task(void *pvParameters)
{

    while (1) {
        // debug_printfln("Motor_Task Running");
        //
        //nInt_Motor_UpdateSpeed(&Left_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Right_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Left_Top_Motor);
        // Int_Motor_UpdateSpeed(&Right_Top_Motor);
        vTaskDelay(100);
    }
}