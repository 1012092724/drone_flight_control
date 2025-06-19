#include "APP_FC_Task.h"

/* 全局变量区 begin */
RC_Status_e RC_Status       = eRC_UNCONNECTED; // 遥控器状态
Drone_Status_e Drone_Status = eDrone_IDLE;     // 无人机状态
/* 全局变量区 end */

// Debug Task
#define Debug_Task_NAME       "Debug_Task"
#define Debug_Task_STACK_SIZE 64
#define Debug_Task_PRIORITY   4
#define Debug_Task_CYCLE      pdMS_TO_TICKS(100)
TaskHandle_t Debug_Task_Handle;
void Debug_Task(void *pvParameters);

// POWER任务
void Power_Task(void *pvParameters);
#define POWER_TASK_NAME       "Power_Task"
#define POWER_TASK_STACK_SIZE 64
#define POWER_TASK_PRIORITY   4
TaskHandle_t Power_Task_Handle;

// LED任务
void LED_Task(void *pvParameters);
#define LED_TASK_NAME       "LED_Task"
#define LED_TASK_STACK_SIZE 64
#define LED_TASK_PRIORITY   2
TaskHandle_t LED_Task_Handle;

// 电机任务
void Motor_Task(void *pvParameters);
#define MOTOR_TASK_NAME       "Motor_Task"
#define MOTOR_TASK_STACK_SIZE 64
#define MOTOR_TASK_PRIORITY   2
TaskHandle_t Motor_Task_Handle;

// 2.4G通讯任务
#define Communication_Task_NAME       "Communication_Task"
#define Communication_Task_STACK_SIZE 128
#define Communication_Task_PRIORITY   3
#define Communication_Task_CYCLE      pdMS_TO_TICKS(10)
TaskHandle_t Communication_Task_Handle;
void Communication_Task(void *pvParameters);

void Sart_ALL_Task()
{
    // 2.4G模块初始化
    App_Communication_Start();
    // 电机初始化
    Int_Motor_Init();
    // 创建Power任务
    xTaskCreate(Power_Task, POWER_TASK_NAME, POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &Power_Task_Handle);
    // 创建Debug任务
    // xTaskCreate(Debug_Task, Debug_Task_NAME, Debug_Task_STACK_SIZE, NULL, Debug_Task_PRIORITY, &Debug_Task_Handle);
    // 创建2.4G通讯任务
    xTaskCreate(Communication_Task, Communication_Task_NAME, Communication_Task_STACK_SIZE, NULL, Communication_Task_PRIORITY, &Communication_Task_Handle);
    // 创建LED任务
    xTaskCreate(LED_Task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_Task_Handle);
    // 创建Motor任务
    xTaskCreate(Motor_Task, MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, &Motor_Task_Handle);
    vTaskStartScheduler(); // Start the FreeRTOS scheduler
}

// Debug任务
void Debug_Task(void *pvParameters)
{
    vTaskDelay(1000);
    debug_printfln("Debug Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    const TickType_t xInterval    = pdMS_TO_TICKS(1000);
    while (1) {
        // printf("PIT:%d, ROL:%d, THR:%d, YAW:%d, FHP:%d, PD:%d \n",
        //        joyStick.PIT, joyStick.ROL, joyStick.THR, joyStick.YAW, joyStick.isFixHeightPoint, joyStick.isPowerDonw);
        vTaskDelayUntil(&pxPreviousWakeTime, xInterval);
    }
}

// 电源控制任务
void Power_Task(void *pvParameters)
{
    debug_printfln("Power Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    const TickType_t xInterval    = pdMS_TO_TICKS(20000);
    vTaskDelay(1500);
    while (1) {
        Int_IP5305T_Open();
        vTaskDelayUntil(&pxPreviousWakeTime, xInterval);
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
        // Int_Motor_UpdateSpeed(&Left_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Right_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Left_Top_Motor);
        // Int_Motor_UpdateSpeed(&Right_Top_Motor);
        Int_Motor_UpdateSpeed(&LeftD_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Right_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Left_Top_Motor);
        // Int_Motor_UpdateSpeed(&Right_Top_Motor);
        vTaskDelay(100);
    }
}

// 2.4G 通讯任务
void Communication_Task(void *pvParameters)
{
    vTaskDelay(1000);
    debug_printfln("Communication Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    Com_Status ReadData           = Com_ERROR;
    while (1) {
        ReadData = App_Communication_ReadRemoteData();
        // debug_printfln("isReadData = %d", isReadData);
        App_Communication_ConnectCheck(ReadData);
        // debug_printfln("staust = %d", staust);

        // 打印接收到的数据
        debug_printfln("PIT:%d, ROL:%d, THR:%d, YAW:%d, FHP:%d, PD:%d",
                       joyStick.PIT, joyStick.ROL, joyStick.THR, joyStick.YAW, joyStick.isFixHeightPoint, joyStick.isPowerDonw);
        xTaskDelayUntil(&pxPreviousWakeTime, Communication_Task_CYCLE);
    }
}
