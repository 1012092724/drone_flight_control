#include "APP_Drone.h"
#include "stdlib.h"
Drone_Status drone_status = Drone_LOCK; // 默认锁定
EulerAngle_Struct eulerAngle;           /* 欧拉角定义 */
Height_Struct height_struct;            /* 高度 */
static int16_t static_AccZ;             // 静态Z轴加速度
static float yaw_lock_target;           // 静态变量保存目标偏航角

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

/*   外环: 俯仰角度  内环: 沿Y轴的角速度*/
PID_Struct pitchPid = {.kp = -6.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};
PID_Struct gyroYPid = {.kp = 5.0f, .ki = 0.0f, .kd = 0.15f, .measure = 0.0f};

/*   外环: 横滚角度  内环: 沿X轴的角速度*/
PID_Struct rollPid  = {.kp = -6.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};
PID_Struct gyroXPid = {.kp = -5.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};

/*   外环: 偏航角度  内环: 沿Z轴的角速度*/
PID_Struct yawPid   = {.kp = -6.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};
PID_Struct gyroZPid = {.kp = -5.0f, .ki = 0.0f, .kd = -0.15f, .measure = 0.0f};

PID_Struct heightPid    = {.kp = -1.2f, .ki = 0, .kd = -0.085f};
PID_Struct velocityYPid = {.kp = -1.2f, .ki = 0, .kd = -0.085f};

/*====================== 6个 姿态PID定义 结束=====================*/

static void APP_Drone_Unlock(void);
static void APP_Drone_Status_Update(float run_cycle);
static void App_Drone_Get_FilteredData(GyroAccel_Struct *gyroAccel);
static void APP_Drone_Attitude_Update(void);
static void App_Drone_GetEulerAngle(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt);
static void App_Drone_Posture_PID(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt);
static void App_Droen_MoveDir_Control(void);
static void App_Drone_Motor_Speed_Update(int16_t input_speed);
static void APP_Drone_Height_PID(Height_Struct *height_struct, float dt);
static void App_Drone_Get_Height_Velocity(Height_Struct *height_struct, float dt);
static int16_t App_Drone_GetNormAccZ(void);

// 无人机初始化
void APP_Drone_Init(void)
{
    Int_Motor_Init();
    Int_MPU6050_Init();
    Int_VL53L1X_Init();
}

void APP_Drone_Start(float run_cycle)
{
    // 无人机状态更新
    APP_Drone_Status_Update();

    // MPU6050 6轴数据滤波
    App_Drone_Get_FilteredData(&gyroAccel);

    // 获取欧拉角
    App_Drone_GetEulerAngle(&gyroAccel, &eulerAngle, run_cycle);

    // 获取高度及垂直方向上的速度
    App_Drone_Get_Height_Velocity(&height_struct, run_cycle);
    // printf("static_accz: %d\n", static_accz);
    // printf("H: %d,V: %f\n", height_struct.height, height_struct.velocity);

    // 6轴数据+欧拉角 进行 姿态PID运算
    App_Drone_Posture_PID(&gyroAccel, &eulerAngle, run_cycle);

    // 无人机姿态更新
    APP_Drone_Attitude_Update(run_cycle);
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

static void APP_Drone_Attitude_Update(float run_cycle)
{
    if (drone_status == Drone_LOCK) {
        // 如果无人机处于锁定状态，保持油门为0
        leftTopMotor.speed =
            leftBottomMotor.speed =
                rightTopMotor.speed =
                    rightBottomMotor.speed = 0;
    } else if (drone_status == Drone_Idle) {
        leftTopMotor.speed =
            rightTopMotor.speed =
                leftBottomMotor.speed =
                    rightBottomMotor.speed = 0;
        // 获取静态状态下垂直方向加速度
        static_AccZ = App_Drone_GetNormAccZ();
        // 空闲时记录当前偏航角为目标锁定角度
        yaw_lock_target = eulerAngle.yaw;
    } else if (drone_status == Drone_NORMAL) {
        // 如果无人机处于正常状态，进行姿态控制
        App_Droen_MoveDir_Control();
        // 电机速度控制 PID
        App_Drone_Motor_Speed_Update(rc_data.THR);
    } else if (drone_status == Drone_HOLD_HIGH) {
        APP_Drone_Height_PID(&heightPid, run_cycle);
        App_Droen_MoveDir_Control();
        App_Drone_Motor_Speed_Update(rc_data.THR);
    } else if (drone_status == Drone_FAULT) {
        leftTopMotor.speed =
            rightTopMotor.speed =
                leftBottomMotor.speed =
                    rightBottomMotor.speed = 0;
    }
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
                drone_status      = Drone_Idle;
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
        case Drone_Idle:
            if (rc_status == RC_UNCONNECTED) {
                drone_status = Drone_FAULT;
            } else {
                if (rc_data.THR > 20) {
                    drone_status = Drone_NORMAL;
                }
            }
            break;
        case Drone_NORMAL:
            if (rc_status == RC_UNCONNECTED) {
                drone_status = Drone_FAULT;
            } else {
                if (rc_data.isFixHeightPoint == 1) {
                    drone_status = Drone_HOLD_HIGH;
                }
                if (rc_data.THR <= 20) {
                    drone_status = Drone_Idle;
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
                drone_status = Drone_Idle;
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

    gyroAccel->gyro.gyroX = Com_Filter_LowPass(gyroAccel->gyro.gyroX, lastGyros[0]);
    gyroAccel->gyro.gyroY = Com_Filter_LowPass(gyroAccel->gyro.gyroY, lastGyros[1]);
    gyroAccel->gyro.gyroZ = Com_Filter_LowPass(gyroAccel->gyro.gyroZ, lastGyros[2]);

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
    gyroAccel->accel.accelX = Com_Filter_KalmanFilter(&accel_kfs[0], gyroAccel->accel.accelX);
    gyroAccel->accel.accelY = Com_Filter_KalmanFilter(&accel_kfs[1], gyroAccel->accel.accelY);
    gyroAccel->accel.accelZ = Com_Filter_KalmanFilter(&accel_kfs[2], gyroAccel->accel.accelZ);

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

static int16_t App_Drone_GetNormAccZ(void)
{
    return Com_Filter_KalmanFilter(&normal_accZ_kfs, Com_IMU_GetNormAccZ());
}

// 获取高度和速度
#define ALPHA 0.985f // IMU 权重，TOF 权重为 1 - ALPHA
static uint16_t tof_heighg  = 0;
static uint16_t sins_height = 0;
static uint16_t last_height = 0;
static float sins_velocityZ = 0.0f;
static float last_velocityZ = 0.0f;
static void App_Drone_Get_Height_Velocity(Height_Struct *height_struct, float dt)
{
    //  获取TOF原始距离
    Int_VL53L1X_Get_Distance(&tof_heighg);

    // 垂直加速度
    int16_t accZ_error = App_Drone_GetNormAccZ() - static_AccZ;

    if (tof_heighg < 3500) { // 此处做一个速度与高度的互补滤波
        if (tof_heighg - sins_height > 50)
            sins_height += 50; // 高度异常突变
        else if (tof_heighg - sins_height < -50)
            sins_height -= 50; // 高度异常突变
        else
            sins_height = tof_heighg;

        // 垂直方向的速度
        sins_velocityZ = (last_velocityZ + accZ_error * dt) * ALPHA + (1 - ALPHA) * (sins_height - last_height) / dt;

        // sins_high= high;
        height_struct->velocity = last_velocityZ = sins_velocityZ;
        height_struct->height = last_height = sins_height;
    }

    // printf("%d,%d,%.2f \n", tof_heighg, height_struct->height, height_struct->velocity);
}
/**
 * @description: 根据欧拉角和采样周期, 进行PID计算
 * @param {GyroAccel_Struct } *gyroAccel
 * @param {EulerAngle_Struct} *eulerAngle
 * @param {float             } dt
 * @return {*}
 */
static void App_Drone_Posture_PID(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt)
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

static void APP_Drone_Height_PID(Height_Struct *height_struct, float dt)
{
    /* 高度 */
    heightPid.dt         = dt;
    heightPid.measure    = height_struct->height;
    velocityYPid.dt      = dt;
    velocityYPid.measure = height_struct->velocity;
    Com_PID_Casecade(&heightPid, &velocityYPid);
}

/**
 * @description: 根据摇杆控制电机运动方向, 就是把要摇杆值作用在角度的期望值
 * @return {*}
 */
static void App_Droen_MoveDir_Control(void)
{
    pitchPid.desire = (rc_data.PIT - 500) * 0.03;
    rollPid.desire  = (rc_data.ROL - 500) * 0.03;

    float yaw_input = (rc_data.YAW - 500) * 0.03;

    if (yaw_input > 1.0f || yaw_input < -1.0f) {
        yaw_lock_target = eulerAngle.yaw + yaw_input;
    }

    yawPid.desire = yaw_lock_target;

    // rollPid.desire = 0;
    // yawPid.desire = 0;
}

/**
 * @description: 根据PID结果控制电机速度更新
 * @return {*}
 */
static void App_Drone_Motor_Speed_Update(int16_t input_speed)
{

    /* 限制油门速度 */
    // uint16_t speed = 0;
    // if (rc_data.isFixHeightPoint) {
    //     speed = LIMIT(rc_data.THRWithHeight, 0, 800);
    // } else {
    //     speed = LIMIT(rc_data.THR, 0, 800);
    // }

    /* 限制油门速度 */
    int16_t speed = LIMIT(input_speed, 0, 600);

    /*
        1. 把pid的结果叠加到油门上 并 限幅
        俯仰: leftTop+rightTop      vs  leftBottom+rightBottom  符号相反   Y轴角速度
        横滚: leftTop+leftBottom    vs  rightTop+rightBottom    符号相反   X轴角速度
        偏航: leftTop+rightBottom   vs  leftBottom+rightTop     符号相反   Z轴角速度
    */
    leftTopMotor.speed     = LIMIT(speed + gyroYPid.result + gyroXPid.result + gyroZPid.result + LIMIT(heightPid.out, -150, 150), 0, 1000);
    rightTopMotor.speed    = LIMIT(speed + gyroYPid.result - gyroXPid.result - gyroZPid.result + LIMIT(heightPid.out, -150, 150), 0, 1000);
    leftBottomMotor.speed  = LIMIT(speed - gyroYPid.result + gyroXPid.result - gyroZPid.result + LIMIT(heightPid.out, -150, 150), 0, 1000);
    rightBottomMotor.speed = LIMIT(speed - gyroYPid.result - gyroXPid.result + gyroZPid.result + LIMIT(heightPid.out, -150, 150), 0, 1000);

    /* 为方便调试,当油门拉到最低时, 所有马达速度置0 */
    // if (rc_data.THR <= 20) {
    //     leftTopMotor.speed =
    //         rightTopMotor.speed =
    //             leftBottomMotor.speed =
    //                 rightBottomMotor.speed = 0;
    // }
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
    // printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",
    //        gyroAccel->gyro.gyroX,
    //        gyroAccel->gyro.gyroY,
    //        gyroAccel->gyro.gyroZ,
    //        gyroAccel->accel.accelX,
    //        gyroAccel->accel.accelY,
    //        gyroAccel->accel.accelZ);
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
