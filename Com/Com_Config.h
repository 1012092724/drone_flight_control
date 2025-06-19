#ifndef __COM_CONFIG_H__
#define __COM_CONFIG_H__

#include "stdint.h"
#include "main.h"

/* 0. 定义通讯的帧头 */
#define FRAME_0 0x20
#define FRAME_1 0x02
#define FRAME_2 0x26

/* 通用状态 */
typedef enum {
    Com_OK = 1,
    Com_ERROR,
    Com_TIMEOUT,
    Com_OTHER
} Com_Status;

/* 定义4个方向 */
typedef enum {
    LEFT_TOP = 0,
    LEFT_BOTTOM,
    RIGHT_TOP,
    RIGHT_BOTTOM
} Location;

/* 封装 Motor 的结构体 */
typedef struct
{
    Location location; /* 马达的位置 */
    int16_t speed;     /* 马达的速度 [0, 1000] */
} Motor_Struct;

/* 封装led的结构体 */
typedef struct
{
    Location location;
    /* 闪烁周期 单位:100ms
        0:表示长亮
        1:表示常灭
        2: 每200ms闪烁一次
        3: 每300ms闪烁一次
        ...
    .*/
    uint8_t blink;
} LED_Struct;

/* 封装陀螺仪 角速度结构体 */
typedef struct
{
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} Gyro_Struct;

/* 封装陀螺仪 加速度结构体 */
typedef struct
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} Accel_Struct;

/* 封装陀螺仪 加速度和角速度结构体 */
typedef struct
{
    Accel_Struct accel;
    Gyro_Struct gyro;
} GyroAccel_Struct;

/* 封装表征飞行器姿态的欧拉角 */
typedef struct
{
    float pitch; /* 俯仰角 */
    float roll;  /* 横滚角 */
    float yaw;   /* 偏航角 */
} EulerAngle_Struct;

/* 通用pid结构体 */
typedef struct
{
    float kp;       /* 比例系数 */
    float ki;       /* 积分系数 */
    float kd;       /* 微分系数 */
    float measure;  /* 测量值 */
    float desire;   /* 期望值 */
    float integral; /* 误差的积分 */
    float lastBias; /* 保存的上次偏差值 */
    float dt;       /* 采样周期 */

    float result; /*  PID的计算结果 */
} PID_Struct;

/*  遥控器数据封装结构体  */
typedef struct
{
    int16_t THR; /* 油门 Throttle */
    int16_t THRWithHeight;

    int16_t YAW; /* 横滚 */
    int16_t ROL; /* 横滚 */
    int16_t PIT; /* 俯仰 */

    uint8_t isFixHeightPoint; /* 是否定高定点 */
    uint8_t isPowerDonw;      /* 是否关机 */
} JoyStick_Struct;

/* 光流计算后最终输出数据结构体 */
typedef struct
{
    float loc_x; /* x位置 */
    float loc_y; /* y位置 */

    float loc_xs; /* x速度 */
    float loc_ys; /* y速度 */
} FlowFinalData_Struct;

extern JoyStick_Struct joyStick; /* 摇杆结构体变量 */

#endif /* __COM_CONFIG_H__ */