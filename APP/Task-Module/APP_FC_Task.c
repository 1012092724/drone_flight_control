#include "APP_FC_Task.h"

// POWER任务
void Power_Task(void *pvParameters);
#define POWER_TASK_NAME       "Power_Task"
#define POWER_TASK_STACK_SIZE 64
#define POWER_TASK_PRIORITY   4
#define POWER_TASK_CYCLE      pdMS_TO_TICKS(10)
TaskHandle_t Power_Task_Handle;

// LED任务
void LED_Task(void *pvParameters);
#define LED_TASK_NAME       "LED_Task"
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY   2
#define LED_TASK_CYCLE      pdMS_TO_TICKS(50)
TaskHandle_t LED_Task_Handle;

// 2.4G通讯任务
#define Communication_Task_NAME       "Communication_Task"
#define Communication_Task_STACK_SIZE 128
#define Communication_Task_PRIORITY   4
#define Communication_Task_CYCLE      pdMS_TO_TICKS(5)
TaskHandle_t Communication_Task_Handle;
void Communication_Task(void *pvParameters);

// 飞控任务
#define Drone_Task_NAME       "Drone_Task"
#define Drone_Task_STACK_SIZE 128
#define Drone_Task_PRIORITY   3
#define Drone_Task_CYCLE      pdMS_TO_TICKS(5)
TaskHandle_t Drone_Task_Handle;
void Drone_Task(void *pvParameters);

// 电机任务
#define Motor_Task_NAME       "Motor_Task"
#define Motor_Task_STACK_SIZE 64
#define Motor_Task_PRIORITY   3
#define Motor_Task_CYCLE      pdMS_TO_TICKS(5)
TaskHandle_t Motor_Task_Handle;
void Motor_Task(void *pvParameters);

void APP_Sart_ALL_Task()
{
    // 2.4G模块初始化
    APP_Communication_Start();
    // Drone初始化
    APP_Drone_Init();
    // 创建 LED任务
    xTaskCreate(LED_Task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &LED_Task_Handle);
    // 创建 Motor任务
    xTaskCreate(Motor_Task, Motor_Task_NAME, Motor_Task_STACK_SIZE, NULL, Motor_Task_PRIORITY, &Motor_Task_Handle);
    // 创建 Drone任务
    xTaskCreate(Drone_Task, Drone_Task_NAME, Drone_Task_STACK_SIZE, NULL, Drone_Task_PRIORITY, &Drone_Task_Handle);
    // 创建 Power任务
    xTaskCreate(Power_Task, POWER_TASK_NAME, POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &Power_Task_Handle);
    // 创建 2.4G通讯任务
    xTaskCreate(Communication_Task, Communication_Task_NAME, Communication_Task_STACK_SIZE, NULL, Communication_Task_PRIORITY, &Communication_Task_Handle);
    vTaskStartScheduler(); // Start the FreeRTOS scheduler
}

// Power 任务
void Power_Task(void *pvParameters)
{
    debug_printfln("Power Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    // vTaskDelay(1000);
    while (1) {
        APP_Power_Control();
        vTaskDelayUntil(&pxPreviousWakeTime, POWER_TASK_CYCLE);
    }
}

// LED 任务
void LED_Task(void *pvParameters)
{
    debug_printfln("LED Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        App_LED_Start();
        vTaskDelayUntil(&pxPreviousWakeTime, LED_TASK_CYCLE);
    }
}

// 2.4G 通讯任务
void Communication_Task(void *pvParameters)
{
    vTaskDelay(1000);
    debug_printfln("Communication Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        // 读取数据
        App_Communication_ReceiveData();
        vTaskDelayUntil(&pxPreviousWakeTime, Communication_Task_CYCLE);
    }
}

// Motor 任务
void Motor_Task(void *pvParameters)
{
    vTaskDelay(1000);
    debug_printfln("Motor Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        APP_Motor_AllWork();
        vTaskDelayUntil(&pxPreviousWakeTime, Motor_Task_CYCLE);
    }
}

// Drone 任务
void Drone_Task(void *pvParameters)
{
    vTaskDelay(1000);
    debug_printfln("Drone Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        APP_Drone_Start(Drone_Task_CYCLE / 1000);
        vTaskDelayUntil(&pxPreviousWakeTime, Drone_Task_CYCLE);
    }
}
