#include "Com_Filter.h"

#define ALPHA 0.15 /* 一阶低通滤波 指数加权系数 */
/**
 * @description: 一阶低通滤波
 *  是一种常用的滤波器，用于去除高频噪声或高频成分，保留信号中的低频成分。
 *  在单片机应用中，一种简单且常见的低通滤波器是一阶无限脉冲响应（IIR）低通滤波器，
 *  通常实现为指数加权移动平均滤波器。
 * @param {int16_t} newValue 需要滤波的值
 * @param {int16_t} preFilteredValue 上一次滤波过的值
 * @return {*}
 */
int16_t Com_Filter_LowPass(int16_t newValue, int16_t preFilteredValue)
{
    return ALPHA * newValue + (1 - ALPHA) * preFilteredValue;
}

/* 卡尔曼滤波 https://www.mwrf.net/tech/basic/2023/30081.html */

/* 卡尔曼滤波参数 */
KalmanFilter_Struct kfs[3] = {
    {0.02f, 0, 0, 0, 0.001f, 0.543f},
    {0.02f, 0, 0, 0, 0.001f, 0.543f},
    {0.02f, 0, 0, 0, 0.001f, 0.543f}};
int16_t Com_Filter_KalmanFilter(KalmanFilter_Struct *kf, int16_t input)
{
    kf->Now_P = kf->LastP + kf->Q;
    kf->Kg    = kf->Now_P / (kf->Now_P + kf->R);
    kf->out   = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->Now_P;
    return kf->out;
}
