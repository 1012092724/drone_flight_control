#include "APP_Drone.h"

Drone_Status drone_status = Drone_LOCK; // 默认锁定
EulerAngle_Struct eulerAngle;           /* 欧拉角定义 */

/* 对x限幅度处理 */
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x)             ((x) >= 0 ? (x) : (-(x)))
/*====================== 6个 姿态 PID定义 开始=====================*/
// /*   外环: 俯仰角度  内环: 沿Y轴的角速度*/
// PID_Struct pitchPid = {.kp = -6.8f, .ki = 0.0f, .kd = 0.0f, .measure = 0.0f};
// PID_Struct gyroYPid = {.kp = 1.7f, .ki = 0.0f, .kd = 0.08f, .measure = 0.0f};

// /*   外环: 横滚角度  内环: 沿X轴的角速度*/
// PID_Struct rollPid  = {.kp = -6.8, .ki = 0, .kd = 0, .measure = 0};
// PID_Struct gyroXPid = {.kp = -1.7, .ki = 0, .kd = -0.08f, .measure = 0};

// /*   外环: 偏航角度  内环: 沿Z轴的角速度*/
// PID_Struct yawPid   = {.kp = -2.5, .ki = 0, .kd = 0, .measure = 0};
// PID_Struct gyroZPid = {.kp = -1, .ki = 0, .kd = 0, .measure = 0};

// /*   外环: 俯仰角度  内环: 沿Y轴的角速度*/
PID_Struct pitchPid = {.kp = -6.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};
PID_Struct gyroYPid = {.kp = 6.0f, .ki = 0.0f, .kd = 0.15f, .measure = 0.0f};

/*   外环: 横滚角度  内环: 沿X轴的角速度*/
PID_Struct rollPid  = {.kp = -6.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};
PID_Struct gyroXPid = {.kp = -6.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};

/*   外环: 偏航角度  内环: 沿Z轴的角速度*/
PID_Struct yawPid   = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .measure = 0.0f};
PID_Struct gyroZPid = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .measure = 0.0f};

/*====================== 6个 姿态PID定义 结束=====================*/

static void APP_Drone_Unlock(void);
static void APP_Drone_Status_Update(void);
static void App_Drone_Get_FilteredData(GyroAccel_Struct *gyroAccel);
static void APP_Drone_Attitude_Update(void);
static void App_Drone_GetEulerAngle(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt);
static void App_Drone_PIDPosture(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt);
static void App_Droen_MoveDir_Control(void);
static void App_Drone_Motor_Speed_Control(int16_t input_speed);

// 无人机初始化
void APP_Drone_Init(void)
{
    Int_Motor_Init();
    Int_MPU6050_Init();
}

void APP_Drone_Start(float run_cycle)
{
    // 无人机状态更新
    APP_Drone_Status_Update();

    // MPU6050 6轴数据滤波
    App_Drone_Get_FilteredData(&gyroAccel);

    // 获取欧拉角
    App_Drone_GetEulerAngle(&gyroAccel, &eulerAngle, run_cycle);

    // 6轴数据+欧拉角 进行PID运算
    App_Drone_PIDPosture(&gyroAccel, &eulerAngle, run_cycle);

    // 无人机姿态更新
    APP_Drone_Attitude_Update();
}

// typedef enum {
//     THR_MAX,
//     THR_FREE,
//     THR_LEAVE_MAX,
//     THR_MIN,
// } THR_Status;

// uint32_t enter_THR_FREE_tick      = 0;
// uint32_t enter_THR_MAX_tick       = 0;
// uint32_t enter_THR_MIN_tick       = 0;
// uint32_t enter_THR_LEAVE_MAX_tick = 0;

// void APP_Flight_Unlock(void)
// {
//     if (rc_status == RC_CONNECTED) {
//         if (flight_status == Flight_LOCK) {
//             switch (thr_status) {
//                 case THR_FREE:
//                     if (rc_data.THR >= 950) {
//                         thr_status         = THR_MAX;
//                         enter_THR_MAX_tick = xTaskGetTickCount();
//                     }
//                     break;
//                 case THR_MAX:
//                     if (rc_data.THR < 950) {
//                         if (xTaskGetTickCount() - enter_THR_MAX_tick >= 1000) {
//                             thr_status               = THR_LEAVE_MAX;
//                             enter_THR_LEAVE_MAX_tick = xTaskGetTickCount();
//                         } else {
//                             thr_status = THR_FREE;
//                         }
//                     }
//                     break;
//                 case THR_LEAVE_MAX:
//                     // 当油门状态处于THR_LEAVE_MAX状态超过2000ms，返回THR_FREE状态
//                     if (xTaskGetTickCount() - enter_THR_LEAVE_MAX_tick > 2000) {
//                         thr_status = THR_FREE;
//                     }
//                     if (rc_data.THR <= 50) {
//                         thr_status         = THR_MIN;
//                         enter_THR_MIN_tick = xTaskGetTickCount();
//                     }
//                     break;
//                 case THR_MIN:
//                     // if (rc_data.THR > 10) {
//                     if (xTaskGetTickCount() - enter_THR_MIN_tick >= 1000) {
//                         thr_status = THR_FREE;
//                         // 更新飞机状态为 解锁
//                         flight_status = Flight_UNLOCK;
//                         // } else {
//                         //     thr_status = THR_FREE;
//                         // }
//                     }
//                     break;
//             }
//         }
//     } else {
//         flight_status = Flight_LOCK;
//         thr_status    = THR_FREE;
//     }
//     printf("thr_status: %d, flight_status: %d\n", thr_status, flight_status);
// }

static void APP_Drone_Attitude_Update(void)
{
    // 更新RC数据到电机速度

    if (drone_status == Drone_LOCK) {
        // 如果无人机处于锁定状态，保持油门为0
        leftTopMotor.speed =
            leftBottomMotor.speed =
                rightTopMotor.speed =
                    rightBotomMotor.speed = 0;
    } else if (drone_status == Drone_NORMAL) {
        // 如果无人机处于正常状态，进行姿态控制
        App_Droen_MoveDir_Control();
        // 电机速度控制 PID
        App_Drone_Motor_Speed_Control(rc_data.THR);

    } else if (drone_status == Drone_HOLD_HIGH) {
        App_Droen_MoveDir_Control();
        App_Drone_Motor_Speed_Control(500);
    } else if (drone_status == Drone_FAULT) {
        // 锁死 状态
        rc_data.PIT = 500;
        rc_data.ROL = 500;
        rc_data.YAW = 500;
        // 进行姿态控制
        App_Droen_MoveDir_Control();
        static uint32_t lastFaultTick = 0;
        if (lastFaultTick == 0) {
            lastFaultTick = xTaskGetTickCount();
        } else if (xTaskGetTickCount() - lastFaultTick > 2000) {
            if (rc_data.THR != 0) {
                /* code */
                rc_data.THR -= 50; // 每次降低50
            }
            lastFaultTick = 0;
        }
        App_Drone_Motor_Speed_Control(rc_data.THR);
    }
    // 更新电机速度 转移到了额外的 任务中
    // Int_Motor_AllWork();
}

static void APP_Drone_Unlock(void)
{
    static uint32_t UnlockFlight_tick = 0;

    // 检查解锁条件：解锁标志为1且油门值为0
    if (rc_data.isUnlockFlight == 1 && rc_data.THR == 0) {
        if (UnlockFlight_tick == 0) {
            // 第一次满足条件，记录当前时间
            UnlockFlight_tick = xTaskGetTickCount();
        } else {
            // 检查是否持续了1.5秒（1500ms）
            if (xTaskGetTickCount() - UnlockFlight_tick >= 1500) {
                drone_status      = Drone_NORMAL;
                UnlockFlight_tick = 0; // 重置计时器
            }
        }
    } else {
        // 条件不满足，重置计时器
        UnlockFlight_tick = 0;
    }
}

static void APP_Drone_Status_Update(void)
{
    switch (drone_status) {
        case Drone_LOCK:
            if (rc_status == RC_CONNECTED) {
                APP_Drone_Unlock();
            }
            break;
        case Drone_NORMAL:
            if (rc_status == RC_UNCONNECTED) {
                drone_status = Drone_FAULT;
            } else {
                if (rc_data.isFixHeightPoint == 1) {
                    drone_status = Drone_HOLD_HIGH;
                }
            }
            break;
        case Drone_HOLD_HIGH:
            if (rc_status == RC_UNCONNECTED) {
                drone_status = Drone_FAULT;
            } else {
                if (rc_data.isFixHeightPoint == 0) {
                    drone_status = Drone_NORMAL;
                }
            }
            break;
        case Drone_FAULT:
            if (rc_status == RC_CONNECTED) {
                drone_status = Drone_NORMAL;
            }
            break;
    }
}

static void App_Drone_Get_FilteredData(GyroAccel_Struct *gyroAccel)
{
    /* 0. 获取mpu原始数据 */
    Int_MPU6050_GetGyroAccel(gyroAccel);

    /* 1. 对角速度使用一阶低通滤波(噪声小) */
    static int32_t lastGyros[3] = {0}; /* 保存上一次的旧值 x y z */
    gyroAccel->gyro.gyroX       = Com_Filter_LowPass(gyroAccel->gyro.gyroX, lastGyros[0]);
    gyroAccel->gyro.gyroY       = Com_Filter_LowPass(gyroAccel->gyro.gyroY, lastGyros[1]);
    gyroAccel->gyro.gyroZ       = Com_Filter_LowPass(gyroAccel->gyro.gyroZ, lastGyros[2]);

    lastGyros[0] = gyroAccel->gyro.gyroX; /* 把这次的保存, 下次使用 */
    lastGyros[1] = gyroAccel->gyro.gyroY;
    lastGyros[2] = gyroAccel->gyro.gyroZ;

    // // 打印原始加速度X轴数据
    // printf("%d,", gyroAccel->accel.accelX);
    // // 对X轴加速度进行一阶滤波
    // static uint16_t TempXAccel;
    // gyroAccel->accel.accelX = Com_Filter_LowPass(gyroAccel->accel.accelX, TempXAccel);
    // TempXAccel              = gyroAccel->accel.accelX;
    // // 打印一阶滤波后的加速度X轴数据
    // printf("%d,", gyroAccel->accel.accelX);

    /* 2. 对加速度使用简易版卡尔曼滤波(噪声大) */
    gyroAccel->accel.accelX = Com_Filter_KalmanFilter(&kfs[0], gyroAccel->accel.accelX);
    gyroAccel->accel.accelY = Com_Filter_KalmanFilter(&kfs[1], gyroAccel->accel.accelY);
    gyroAccel->accel.accelZ = Com_Filter_KalmanFilter(&kfs[2], gyroAccel->accel.accelZ);

    // // 打印卡尔曼滤波后的加速度X轴数据
    // printf("%d\n", gyroAccel->accel.accelX);
}

/**
 * @description: 根据mpu的6轴数据, 获取表征姿态的欧拉角
 * @param {GyroAccel_Struct} *gyroAccel mpu的6轴数据
 * @param {EulerAngle_Struct} *EulerAngle 计算后得到的欧拉角
 * @param {float} dt 采样周期 (单位s)
 * @return {*}
 */
static void App_Drone_GetEulerAngle(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt)
{
    Com_IMU_GetEulerAngle(gyroAccel, eulerAngle, dt);
}

/**
 * @description: 获取Z轴上的加速度 (如果已经倾斜,会考虑z轴上加速度的合成)
 * @return {*}
 */
static float App_Drone_GetNormAccZ(void)
{
    return Com_IMU_GetNormAccZ();
}

/**
 * @description: 根据欧拉角和采样周期, 进行PID计算
 * @param {GyroAccel_Struct } *gyroAccel
 * @param {EulerAngle_Struct} *eulerAngle
 * @param {float             } dt
 * @return {*}
 */
static void App_Drone_PIDPosture(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt)
{
    /* 俯仰 */
    pitchPid.dt      = dt;
    pitchPid.measure = eulerAngle->pitch; /* 外 角度 */
    gyroYPid.dt      = dt;
    gyroYPid.measure = gyroAccel->gyro.gyroY * Gyro_G; /* 内 角速度 */
    Com_PID_Casecade(&pitchPid, &gyroYPid);

    /* 横滚 */
    rollPid.dt       = dt;
    rollPid.measure  = eulerAngle->roll;
    gyroXPid.dt      = dt;
    gyroXPid.measure = gyroAccel->gyro.gyroX * Gyro_G;
    Com_PID_Casecade(&rollPid, &gyroXPid);

    /* 偏航 */
    yawPid.dt        = dt;
    yawPid.measure   = eulerAngle->yaw;
    gyroZPid.dt      = dt;
    gyroZPid.measure = gyroAccel->gyro.gyroZ * Gyro_G;
    Com_PID_Casecade(&yawPid, &gyroZPid);
}

/**
 * @description: 根据摇杆控制电机运动方向, 就是把要摇杆值作用在角度的期望值
 * @return {*}
 */
static void App_Droen_MoveDir_Control(void)
{

    /* 我们的值是 0-1000 减去500 之后,变成-500到+500 */
    pitchPid.desire = (rc_data.PIT - 500) * 0.03;
    rollPid.desire  = (rc_data.ROL - 500) * 0.03;
    yawPid.desire   = (rc_data.YAW - 500) * 0.03;
}

/**
 * @description: 根据PID结果控制电机运动
 * @return {*}
 */
static void App_Drone_Motor_Speed_Control(int16_t input_speed)
{

    /* 限制油门速度 */
    // uint16_t speed = 0;
    // if (rc_data.isFixHeightPoint) {
    //     speed = LIMIT(rc_data.THRWithHeight, 0, 800);
    // } else {
    //     speed = LIMIT(rc_data.THR, 0, 800);
    // }

    /* 限制油门速度 */
    uint16_t speed = LIMIT(input_speed, 0, 700);

    /*
        1. 把pid的结果叠加到油门上
        俯仰: leftTop+rightTop      vs  leftBottom+rightBottom  符号相反   Y轴角速度
        横滚: leftTop+leftBottom    vs  rightTop+rightBottom    符号相反   X轴角速度
        偏航: leftTop+rightBottom   vs  leftBottom+rightTop     符号相反   Z轴角速度
    */
    leftTopMotor.speed    = speed + gyroYPid.result + gyroXPid.result + gyroZPid.result;
    rightTopMotor.speed   = speed + gyroYPid.result - gyroXPid.result - gyroZPid.result;
    leftBottomMotor.speed = speed - gyroYPid.result + gyroXPid.result - gyroZPid.result;
    rightBotomMotor.speed = speed - gyroYPid.result - gyroXPid.result + gyroZPid.result;

    /* 对叠加后的结果做限幅 */
    leftTopMotor.speed    = LIMIT(leftTopMotor.speed, 0, 1000);
    rightTopMotor.speed   = LIMIT(rightTopMotor.speed, 0, 1000);
    leftBottomMotor.speed = LIMIT(leftBottomMotor.speed, 0, 1000);
    rightBotomMotor.speed = LIMIT(rightBotomMotor.speed, 0, 1000);

    /* 为方便调试,当油门拉到最低时, 所有马达速度置0 */
    if (rc_data.THR <= 20) {
        leftTopMotor.speed =
            rightTopMotor.speed =
                leftBottomMotor.speed =
                    rightBotomMotor.speed = 0;
    }
}

/* =================================用于测试打印的函数============================ */
/**
 * @description: 打印MPU6050数据, 方便调试
 * @param {uint8_t} *pre  打印的前缀表示
 * @param {GyroAccel_Struct} *gyroAccel mpu6050数据结构体
 * @return {*}
 */
void printGyroAccel(GyroAccel_Struct *gyroAccel)
{
    printf("%d,%d,%d,%d,%d,%d,",
           gyroAccel->gyro.gyroX,
           gyroAccel->gyro.gyroY,
           gyroAccel->gyro.gyroZ,
           gyroAccel->accel.accelX,
           gyroAccel->accel.accelY,
           gyroAccel->accel.accelZ);
}

/**
 * @description: 输出欧拉角
 * @param {EulerAngle_Struct} eulerAngle
 * @return {*}
 */
void printfEulerAngle(EulerAngle_Struct *eulerAngle)
{
    printf("%f,%f,%f\n",
           eulerAngle->pitch,
           eulerAngle->roll,
           eulerAngle->yaw);
}
