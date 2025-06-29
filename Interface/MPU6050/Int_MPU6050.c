#include "Int_MPU6050.h"
#include "stdlib.h"

GyroAccel_Struct gyroAccel;



#define I2C_DELAY 1000
// #define abs(x)    ((x) >= 0 ? (x) : (-x))

static void
Int_MPU6050_WriteReg(uint8_t reg, uint8_t value);
static uint8_t Int_MPU6050_ReadReg(uint8_t reg);
static void Int_MPU6050_ReadRegs(uint8_t regAddr, uint8_t data[], uint8_t len);
static void Int_MPU6050_Calibarate(void);

/* 存储6个偏移值 */
static GyroAccel_Struct mpuOffset;
static uint8_t isCalibrated = 0; /* 1 表示是否已经校准完成 */
/**
 * @description: 读取设备id, 用于检测驱动是否正常
 * @return {*} 读取到的id值. 根据寄存器描述,读取到的其实就是它的7位I2C地址0x68
 */
uint8_t Int_MPU6050_ReadId(void)
{
    return Int_MPU6050_ReadReg(MPU_DEVICE_ID_REG);
}

/**
 * @description: 初始化MPU6050芯片
 * @return {*}
 */
void Int_MPU6050_Init(void)
{
    /* 1.  复位 =>休眠=>唤醒*/
    Int_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 0x80);
    HAL_Delay(1000); /* 延时150ms再唤醒 */
    /*唤醒: 复位后自动进入睡眠模式,所以需要先唤醒 */
    Int_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 0x00);

    /* 2. 设置量程 */
    /* 2.1 设置角速度量程 ±2000°/s*/
    Int_MPU6050_WriteReg(MPU_GYRO_CFG_REG, 0x18); // 0001 1000 0x18
    /* 2.2 设置加速度量程 ±2g*/
    Int_MPU6050_WriteReg(MPU_ACCEL_CFG_REG, 0 << 3);

    /* 3. 其他配置: 关闭FIFO 中断 第二I2C */
    Int_MPU6050_WriteReg(MPU_USER_CTRL_REG, 0x00);
    Int_MPU6050_WriteReg(MPU_FIFO_EN_REG, 0x00);
    Int_MPU6050_WriteReg(MPU_INT_EN_REG, 0x00);

    /* 4. 设置设备的系统时钟源 PLL参考陀螺仪x轴*/
    Int_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 0x01);

    /* 5. 设置低通滤波器和采样率 */ // 低通滤波器只开1档,如果开太高 延迟太高
    /* 5.1 设置采样率 500Hz = 1000/(1 + div) => div = 1 */
    // Int_MPU6050_WriteReg(MPU_SAMPLE_RATE_REG, 1);
    // 可以不分频
    Int_MPU6050_WriteReg(MPU_SAMPLE_RATE_REG, 0x00);

    /* 5.2 设置低通滤波器   香农采样定律: 采样率>=2*最高频率   最高频率<=采样率/2  最高频率<=250 */
    Int_MPU6050_WriteReg(MPU_CFG_REG, 1 << 0);

    /* 6. 启动加速度传感器和陀螺仪 */ // 待机位寄存器 1 待机
    Int_MPU6050_WriteReg(MPU_PWR_MGMT2_REG, 0x00);

    /* 7. 对陀螺仪做校准 */
    isCalibrated = 0;
    Int_MPU6050_Calibarate();
}

// 通过I2C对MPU6050内的某个寄存器进行写操作
void Int_MPU6050_WriteReg(uint8_t regAddr, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, (uint16_t)regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_DELAY);
}

// 通过I2C对MPU6050内的寄存器进行连续读操作
static uint8_t Int_MPU6050_ReadReg(uint8_t regAddr)
{
    uint8_t res = 0x00;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, (uint16_t)regAddr, I2C_MEMADD_SIZE_8BIT, &res, 1, I2C_DELAY);
    return res;
}

/**
 * @description: 从指定的寄存器开始连续读取多个寄存器的值
 * @param {uint8_t} reg 开始的寄存器地址
 * @param {uint8_t} len 读取的寄存器的个数
 * @param {uint8_t} data 存储读取到值
 * @return {*}
 */
static void Int_MPU6050_ReadRegs(uint8_t regAddr, uint8_t data[], uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, (uint16_t)regAddr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_DELAY);
}

/**
 * @description: 获取陀螺仪的各轴的角速度
 * @param {Gyro_Struct} *gyro
 * @return {*}
 */
void Int_MPU6050_GetGyro(Gyro_Struct *gyro)
{
    uint8_t buff[6];
    Int_MPU6050_ReadRegs(MPU_GYRO_XOUTH_REG, buff, 6);
    gyro->gyroX = (int16_t)(buff[0] << 8 | buff[1]);
    gyro->gyroY = (int16_t)(buff[2] << 8 | buff[3]);
    gyro->gyroZ = (int16_t)(buff[4] << 8 | buff[5]);
}

/**
 * @description: 获取陀螺仪的各轴的加速度
 * @param {Accel_Struct} *accel
 * @return {*}
 */
void Int_MPU6050_GetAccel(Accel_Struct *accel)
{
    uint8_t buff[6];
    Int_MPU6050_ReadRegs(MPU_ACCEL_XOUTH_REG, buff, 6);
    accel->accelX = (int16_t)(buff[0] << 8 | buff[1]);
    accel->accelY = (int16_t)(buff[2] << 8 | buff[3]);
    accel->accelZ = (int16_t)(buff[4] << 8 | buff[5]);
}

/**
 * @description: 获取陀螺仪各轴的加速度和角速度
 * @param {GyroAccel_Struct} *gyroAccel
 * @return {*}
 */
void Int_MPU6050_GetGyroAccel(GyroAccel_Struct *gyroAccel)
{
    Int_MPU6050_GetGyro(&gyroAccel->gyro);
    Int_MPU6050_GetAccel(&gyroAccel->accel);

    if (isCalibrated) /* 判断是否校准过 */
    {
        /* 对值做校准 */
        gyroAccel->accel.accelX -= mpuOffset.accel.accelX;
        gyroAccel->accel.accelY -= mpuOffset.accel.accelY;
        gyroAccel->accel.accelZ -= mpuOffset.accel.accelZ;

        gyroAccel->gyro.gyroX -= mpuOffset.gyro.gyroX;
        gyroAccel->gyro.gyroY -= mpuOffset.gyro.gyroY;
        gyroAccel->gyro.gyroZ -= mpuOffset.gyro.gyroZ;
    }
}

/**
 * @description: 在陀螺仪静止条件下,对陀螺仪做校准
 *  在正常情况下, MPU6050水平静止放置时:
 *          沿x,y,z各轴的角速度为0
 *          沿x,y 方向的加速度为0
 *          沿z方向的加速是16383
 *  校准的过程就是读出水平静止放置时的6个值,将来获取这个6个值时,需要减去这6个值
 * @return {*}
 */
static void Int_MPU6050_Calibarate(void)
{

    /* 1. 等待 MPU6050 处于静止状态
           a:判断沿着x,y,z 的角速度本次和上次的差值小于1个阈值,就认为是静止状态
           为了减少误差,连续判断100次.   只判断角速度即可
           b:采样率设置的为1000Hz, 所以两次读取的间隔不能低于1ms
     */
    int16_t gyroThreshHold  = 50;  /* 静止阈值 */
    Gyro_Struct lastGyro    = {0}; /* 存储上次读取到的角速度 */
    Gyro_Struct currentGryo = {0}; /* 存储这次读取到的角速度 */
    Int_MPU6050_GetGyro(&lastGyro);
    int16_t cnt = 400;

    // 定义一个偏移缓冲区
    int32_t buff[6] = {0}; /* 存储6个量的和 */
    while (cnt > 0) {
        Int_MPU6050_GetGyro(&currentGryo);
        int16_t deltaGyroX = abs(currentGryo.gyroX - lastGyro.gyroX);
        int16_t deltaGyroY = abs(currentGryo.gyroY - lastGyro.gyroY);
        int16_t deltaGyroZ = abs(currentGryo.gyroZ - lastGyro.gyroZ);
        if (deltaGyroX <= gyroThreshHold && deltaGyroY <= gyroThreshHold && deltaGyroZ <= gyroThreshHold) {
            /* 如果当前的角速度和上次的角速度差值小于阈值, 就认为是静止状态 */
            /* 读取陀螺仪和加速度计的值, 并累加到buff中 */
            Int_MPU6050_GetGyroAccel(&mpuOffset);
            buff[0] += mpuOffset.gyro.gyroX - 0;
            buff[1] += mpuOffset.gyro.gyroY - 0;
            buff[2] += mpuOffset.gyro.gyroZ - 0;
            buff[3] += mpuOffset.accel.accelX - 0;
            buff[4] += mpuOffset.accel.accelY - 0;
            buff[5] += mpuOffset.accel.accelZ - 16383;
            cnt--;

        } else {
            /* 如果当前的角速度和上次的角速度差值大于阈值, 就认为不是静止状态 */
            // 清空缓存区buff[6] 最快的方法
            buff[0] = 0;
            buff[1] = 0;
            buff[2] = 0;
            buff[3] = 0;
            buff[4] = 0;
            buff[5] = 0;
            cnt     = 400; /* 一旦出现非静止状态, 把cnt的值重新初始化为 100 */
        }
        /* 把current值,赋值给last, 方便下次使用*/
        // lastGyro.gyroX = currentGryo.gyroX;
        // lastGyro.gyroY = currentGryo.gyroY;
        // lastGyro.gyroZ = currentGryo.gyroZ;
        lastGyro = currentGryo;
        HAL_Delay(1); // 采样率设置的为1000Hz, 所以两次读取的间隔不能低于1ms
    }

    // /* 2. 读出6个偏移量: 多次读取, 最后取平均值 */
    // int32_t buff[6] = {0}; /* 存储6个量的和 */
    // for (uint16_t i = 0; i < 256; i++) {
    //     Int_MPU6050_GetGyroAccel(&mpuOffset);
    //     buff[0] += mpuOffset.gyro.gyroX - 0;
    //     buff[1] += mpuOffset.gyro.gyroY - 0;
    //     buff[2] += mpuOffset.gyro.gyroZ - 0;

    //     buff[3] += mpuOffset.accel.accelX - 0;
    //     buff[4] += mpuOffset.accel.accelY - 0;
    //     buff[5] += mpuOffset.accel.accelZ - 16383;

    //     HAL_Delay(3); /* 每3ms读取一次 */
    // }
    mpuOffset.gyro.gyroX = (buff[0] / 400); /* 除以256或者右移8位 */
    mpuOffset.gyro.gyroY = (buff[1] / 400);
    mpuOffset.gyro.gyroZ = (buff[2] / 400);

    mpuOffset.accel.accelX = (buff[3] / 400);
    mpuOffset.accel.accelY = (buff[4] / 400);
    mpuOffset.accel.accelZ = (buff[5] / 400);

    // 修改校准状态标志位
    isCalibrated = 1;
}
