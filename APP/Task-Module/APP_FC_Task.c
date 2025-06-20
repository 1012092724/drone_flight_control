#include "APP_FC_Task.h"

/* 全局变量区 begin */
Drone_Status drone_status = eDrone_IDLE; // 无人机状态
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
#define POWER_TASK_CYCLE      pdMS_TO_TICKS(20000)
TaskHandle_t Power_Task_Handle;

// LED任务
void LED_Task(void *pvParameters);
#define LED_TASK_NAME       "LED_Task"
#define LED_TASK_STACK_SIZE 64
#define LED_TASK_PRIORITY   2
#define LED_TASK_CYCLE      pdMS_TO_TICKS(100)
TaskHandle_t LED_Task_Handle;

// 电机任务
void Motor_Task(void *pvParameters);
#define MOTOR_TASK_NAME       "Motor_Task"
#define MOTOR_TASK_STACK_SIZE 64
#define MOTOR_TASK_PRIORITY   3
#define MOTOR_TASK_CYCLE      pdMS_TO_TICKS(10)
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
    xTaskCreate(Debug_Task, Debug_Task_NAME, Debug_Task_STACK_SIZE, NULL, Debug_Task_PRIORITY, &Debug_Task_Handle);
    // 创建2.4G通讯任务
    xTaskCreate(Communication_Task, Communication_Task_NAME, Communication_Task_STACK_SIZE, NULL, Communication_Task_PRIORITY, &Communication_Task_Handle);
    // 创建LED任务
    xTaskCreate(LED_Task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_Task_Handle);
    // 创建Motor任务
    // xTaskCreate(Motor_Task, MOTOR_TASK_NAME, MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, &Motor_Task_Handle);
    vTaskStartScheduler(); // Start the FreeRTOS scheduler
}

// Debug任务
void Debug_Task(void *pvParameters)
{
    vTaskDelay(1000);
    debug_printfln("Debug Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    const TickType_t xInterval    = pdMS_TO_TICKS(100);
    while (1) {
        if (rc_status == RC_CONNECTED) {
            printf("PIT:%d, ROL:%d, THR:%d, YAW:%d, FHP:%d, PD:%d \n",
                   joyStick.PIT, joyStick.ROL, joyStick.THR, joyStick.YAW, joyStick.isFixHeightPoint, joyStick.isPowerDonw);
        }

        vTaskDelayUntil(&pxPreviousWakeTime, xInterval);
    }
}

// 电源控制任务
void Power_Task(void *pvParameters)
{
    debug_printfln("Power Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    vTaskDelay(1500);
    while (1) {
        APP_Power_Open();
        vTaskDelayUntil(&pxPreviousWakeTime, POWER_TASK_CYCLE);
    }
}

void LED_Task(void *pvParameters)
{
    debug_printfln("LED Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    // uint16_t last_toggle_time     = 0; // 上一次切换LED的时间
    while (1) {
        App_LED_Start();
        xTaskDelayUntil(&pxPreviousWakeTime, LED_TASK_CYCLE);
    }
}

void Motor_Task(void *pvParameters)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        // debug_printfln("Motor_Task Running");
        // Int_Motor_UpdateSpeed(&Left_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Right_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Left_Top_Motor);
        // Int_Motor_UpdateSpeed(&Right_Top_Motor);
        // Int_Motor_UpdateSpeed(&LeftD_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&leftTopMotor);
        // Int_Motor_UpdateSpeed(&Right_Bottom_Motor);
        // Int_Motor_UpdateSpeed(&Left_Top_Motor);
        // Int_Motor_UpdateSpeed(&Right_Top_Motor);
        xTaskDelayUntil(&pxPreviousWakeTime, MOTOR_TASK_CYCLE);
    }
}

// 2.4G 通讯任务
void Communication_Task(void *pvParameters)
{
    vTaskDelay(1000);
    debug_printfln("Communication Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    Data_Status ReadData          = Data_Invalid; // 初始化读取数据状态为无效
    while (1) {
        // 读取数据
        ReadData = App_Communication_ValidatePacket();
        App_Communication_UpdateConnectionStatus(ReadData);
        xTaskDelayUntil(&pxPreviousWakeTime, Communication_Task_CYCLE);
    }
}
