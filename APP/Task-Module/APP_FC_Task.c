#include "APP_FC_Task.h"
#include <string.h>
#include <stdlib.h>

// POWER任务
void Power_Task(void *pvParameters);
#define POWER_TASK_NAME       "Power_Task"
#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY   4
#define POWER_TASK_CYCLE      pdMS_TO_TICKS(20)
TaskHandle_t Power_Task_Handle;

// LED任务
void LED_Task(void *pvParameters);
#define LED_TASK_NAME       "LED_Task"
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY   2
#define LED_TASK_CYCLE      pdMS_TO_TICKS(40)
TaskHandle_t LED_Task_Handle;

// 2.4G通讯任务
#define Communication_Task_NAME       "Communication_Task"
#define Communication_Task_STACK_SIZE 128
#define Communication_Task_PRIORITY   3
#define Communication_Task_CYCLE      pdMS_TO_TICKS(10)
TaskHandle_t Communication_Task_Handle;
void Communication_Task(void *pvParameters);

// 飞控任务
#define Drone_Task_NAME       "Drone_Task"
#define Drone_Task_STACK_SIZE 512
#define Drone_Task_PRIORITY   2
#define Drone_Task_CYCLE      pdMS_TO_TICKS(2)
TaskHandle_t Drone_Task_Handle;
void Drone_Task(void *pvParameters);

// 电机任务
#define Motor_Task_NAME       "Motor_Task"
#define Motor_Task_STACK_SIZE 128
#define Motor_Task_PRIORITY   2
#define Motor_Task_CYCLE      pdMS_TO_TICKS(2)
TaskHandle_t Motor_Task_Handle;
void Motor_Task(void *pvParameters);

// debug 任务
#define Debug_Task_NAME       "Debug_Task"
#define Debug_Task_STACK_SIZE 512
#define Debug_Task_PRIORITY   3
#define Debug_Task_CYCLE      pdMS_TO_TICKS(100)
TaskHandle_t Debug_Task_Handle;
void Debug_Task(void *pvParameters);

// PID Debug
#define PID_Debug_Task_NAME       "PID_Debug_Task"
#define PID_Debug_Task_STACK_SIZE 512
#define PID_Debug_Task_PRIORITY   2
#define PID_Debug_Task_CYCLE      pdMS_TO_TICKS(15)
TaskHandle_t PID_Debug_Task_Handle;
void PID_Debug_Task(void *pvParameters);

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

    // 创建DEBUG
    // xTaskCreate(Debug_Task, Debug_Task_NAME, Debug_Task_STACK_SIZE, NULL, Debug_Task_PRIORITY, &Debug_Task_Handle);

    // 创建PID debug
    // printf("%.4f,%.4f,%.4f,", pitchPid.kp, pitchPid.ki, pitchPid.kd);
    // printf("%.4f,%.4f,%.4f,", rollPid.kp, rollPid.ki, rollPid.kd);
    // printf("%.4f,%.4f,%.4f,", yawPid.kp, yawPid.ki, yawPid.kd);
    // printf("%.4f,%.4f,%.4f,", gyroXPid.kp, gyroXPid.ki, gyroXPid.kd);
    // printf("%.4f,%.4f,%.4f,", gyroYPid.kp, gyroYPid.ki, gyroYPid.kd);
    // printf("%.4f,%.4f,%.4f\n", gyroZPid.kp, gyroZPid.ki, gyroZPid.kd);
    // xTaskCreate(PID_Debug_Task, PID_Debug_Task_NAME, PID_Debug_Task_STACK_SIZE, NULL, PID_Debug_Task_PRIORITY, &PID_Debug_Task_Handle);
    vTaskStartScheduler(); // Start the FreeRTOS scheduler
}

void Debug_Task(void *pvParameters)
{
    extern GyroAccel_Struct gyroAccel;
    extern EulerAngle_Struct eulerAngle;
    vTaskDelay(2000);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        printGyroAccel(&gyroAccel);
        printfEulerAngle(&eulerAngle);
        // 打印 PID
        // printf("%.4f,%.4f,%.4f\n",

        //        //    pitchPid.kp, pitchPid.ki, pitchPid.kd,

        //        //    rollPid.kp, rollPid.ki, rollPid.kd,

        //        //    yawPid.kp, yawPid.ki, yawPid.kd,

        //        gyroXPid.kp, gyroXPid.ki, gyroXPid.kd,

        //        //    gyroYPid.kp, gyroYPid.ki, gyroYPid.kd,

        //        //    gyroZPid.kp, gyroZPid.ki, gyroZPid.kd
        // );

        // printf("%.4f,%.4f,%.4f,", pitchPid.kp, pitchPid.ki, pitchPid.kd);
        // printf("%.4f,%.4f,%.4f,", rollPid.kp, rollPid.ki, rollPid.kd);
        // printf("%.4f,%.4f,%.4f,", yawPid.kp, yawPid.ki, yawPid.kd);
        // printf("%.4f,%.4f,%.4f,", gyroXPid.kp, gyroXPid.ki, gyroXPid.kd);
        // printf("%.4f,%.4f,%.4f,", gyroYPid.kp, gyroYPid.ki, gyroYPid.kd);
        // printf("%.4f,%.4f,%.4f\n", gyroZPid.kp, gyroZPid.ki, gyroZPid.kd);

        vTaskDelayUntil(&pxPreviousWakeTime, Debug_Task_CYCLE);
    }
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
    vTaskDelay(2000);
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
    vTaskDelay(2000);
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
    vTaskDelay(2000);
    debug_printfln("Drone Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        // printf("%d\n", xTaskGetTickCount());
        APP_Drone_Start((float)(Drone_Task_CYCLE / 1000.0));
        // printf("%d\n", xTaskGetTickCount());
        vTaskDelayUntil(&pxPreviousWakeTime, Drone_Task_CYCLE);
    }
}

// // PID Debug
// #define PID_Debug_Task_NAME       "PID_Debug_Task"
// #define PID_Debug_Task_STACK_SIZE 512
// #define PID_Debug_Task_PRIORITY   4
// #define PID_Debug_Task_CYCLE      pdMS_TO_TICKS(100)
// TaskHandle_t PID_Debug_Task_Handle;
// void PID_Debug_Task(void *pvParameters);

#define PID_DEBUG_BUFFER_SIZE 25

void PID_Debug_Task(void *pvParameters)
{
    // 接收缓冲区
    static char pid_debug_buffer[PID_DEBUG_BUFFER_SIZE] = {0};
    static uint16_t pid_debug_buffer_len                = 0;

    vTaskDelay(2000);
    debug_printfln("PID Debug Task: Start!");
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    while (1) {
        HAL_UARTEx_ReceiveToIdle(&huart2, (uint8_t *)pid_debug_buffer, PID_DEBUG_BUFFER_SIZE, &pid_debug_buffer_len, 50);
        if (pid_debug_buffer_len > 0) {
            // 检查数据完整性（以 '!' 结尾）
            if (pid_debug_buffer[pid_debug_buffer_len - 1] == '!') {
                pid_debug_buffer[pid_debug_buffer_len] = '\0'; // 确保字符串结束

                if (pid_debug_buffer[0] != 'g') {
                    switch (pid_debug_buffer[0]) {
                        case 'p': // pitch
                            if (pid_debug_buffer[1] == 'p' || pid_debug_buffer[1] == 'i' || pid_debug_buffer[1] == 'd') {
                                pid_debug_buffer[2] = '\0';
                                char *endptr;
                                float value = strtof(pid_debug_buffer + 3, &endptr);
                                if (endptr != pid_debug_buffer + 3 && *endptr == '!') {
                                    switch (pid_debug_buffer[1]) {
                                        case 'p':
                                            pitchPid.kp = value;
                                            break;
                                        case 'i':
                                            pitchPid.ki = value;
                                            break;
                                        case 'd':
                                            pitchPid.kd = value;
                                            break;
                                    }
                                }
                            }
                            break;
                        case 'r': // roll
                            if (pid_debug_buffer[1] == 'p' || pid_debug_buffer[1] == 'i' || pid_debug_buffer[1] == 'd') {
                                pid_debug_buffer[2] = '\0';
                                char *endptr;
                                float value = strtof(pid_debug_buffer + 3, &endptr);
                                if (endptr != pid_debug_buffer + 3 && *endptr == '!') {
                                    switch (pid_debug_buffer[1]) {
                                        case 'p':
                                            rollPid.kp = value;
                                            break;
                                        case 'i':
                                            rollPid.ki = value;
                                            break;
                                        case 'd':
                                            rollPid.kd = value;
                                            break;
                                    }
                                }
                            }
                            break;
                        case 'y': // yaw
                            if (pid_debug_buffer[1] == 'p' || pid_debug_buffer[1] == 'i' || pid_debug_buffer[1] == 'd') {
                                pid_debug_buffer[2] = '\0';
                                char *endptr;
                                float value = strtof(pid_debug_buffer + 3, &endptr);
                                if (endptr != pid_debug_buffer + 3 && *endptr == '!') {
                                    switch (pid_debug_buffer[1]) {
                                        case 'p':
                                            yawPid.kp = value;
                                            break;
                                        case 'i':
                                            yawPid.ki = value;
                                            break;
                                        case 'd':
                                            yawPid.kd = value;
                                            break;
                                    }
                                }
                            }
                            break;
                    }
                } else {
                    if (pid_debug_buffer[1] == 'x' || pid_debug_buffer[1] == 'y' || pid_debug_buffer[1] == 'z') {
                        if (pid_debug_buffer[2] == 'p' || pid_debug_buffer[2] == 'i' || pid_debug_buffer[2] == 'd') {
                            pid_debug_buffer[3] = '\0';
                            char *endptr;
                            float value = strtof(pid_debug_buffer + 4, &endptr);
                            if (endptr != pid_debug_buffer + 4 && *endptr == '!') {
                                switch (pid_debug_buffer[1]) {
                                    case 'x':
                                        switch (pid_debug_buffer[2]) {
                                            case 'p':
                                                gyroXPid.kp = value;
                                                break;
                                            case 'i':
                                                gyroXPid.ki = value;
                                                break;
                                            case 'd':
                                                gyroXPid.kd = value;
                                                break;
                                        }
                                        break;
                                    case 'y':
                                        switch (pid_debug_buffer[2]) {
                                            case 'p':
                                                gyroYPid.kp = value;
                                                break;
                                            case 'i':
                                                gyroYPid.ki = value;
                                                break;
                                            case 'd':
                                                gyroYPid.kd = value;
                                                break;
                                        }
                                        break;
                                    case 'z':
                                        switch (pid_debug_buffer[2]) {
                                            case 'p':
                                                gyroZPid.kp = value;
                                                break;
                                            case 'i':
                                                gyroZPid.ki = value;
                                                break;
                                            case 'd':
                                                gyroZPid.kd = value;
                                                break;
                                        }
                                        break;
                                }
                            }
                        }
                    }
                }

                pid_debug_buffer[0]  = '\0'; // 清空缓冲区
                pid_debug_buffer_len = 0;
                printf("%.4f,%.4f,%.4f,", pitchPid.kp, pitchPid.ki, pitchPid.kd);
                printf("%.4f,%.4f,%.4f,", rollPid.kp, rollPid.ki, rollPid.kd);
                printf("%.4f,%.4f,%.4f,", yawPid.kp, yawPid.ki, yawPid.kd);
                printf("%.4f,%.4f,%.4f,", gyroXPid.kp, gyroXPid.ki, gyroXPid.kd);
                printf("%.4f,%.4f,%.4f,", gyroYPid.kp, gyroYPid.ki, gyroYPid.kd);
                printf("%.4f,%.4f,%.4f\n", gyroZPid.kp, gyroZPid.ki, gyroZPid.kd);
            }
        }
        vTaskDelayUntil(&pxPreviousWakeTime, PID_Debug_Task_CYCLE);
    }
}