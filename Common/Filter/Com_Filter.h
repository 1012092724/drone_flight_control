#ifndef __COM_FILTER_H__
#define __COM_FILTER_H__

#include "Com_Config.h"

/* 卡尔曼滤波器结构体 */
typedef struct
{
    float LastP; // 上一时刻的状态方差（或协方差）
    float Now_P; // 当前时刻的状态方差（或协方差）
    float out;   // 滤波器的输出值，即估计的状态
    float Kg;    // 卡尔曼增益，用于调节预测值和测量值之间的权重
    float Q;     // 过程噪声的方差，反映系统模型的不确定性
    float R;     // 测量噪声的方差，反映测量过程的不确定性
} KalmanFilter_Struct;

extern KalmanFilter_Struct accel_kfs[3];
extern KalmanFilter_Struct normal_accZ_kfs;

int16_t Com_Filter_LowPass(int16_t newValue, int16_t preFilteredValue);

int16_t Com_Filter_KalmanFilter(KalmanFilter_Struct *kf, int16_t input);

#endif /* __COM_FILTER_H__ */