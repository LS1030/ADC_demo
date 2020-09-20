#include "KalmanFilter.h"

KalmanFilter_TypeDef ADC1_IN1_KalmanFilter;

void ADC_KalmanFilter_Init(void)
{
    ADC1_IN1_KalmanFilter.LastP = 0.02;
    ADC1_IN1_KalmanFilter.Now_P = 0.0;
    ADC1_IN1_KalmanFilter.out = 0.0;
    ADC1_IN1_KalmanFilter.Kg = 0.0;
    ADC1_IN1_KalmanFilter.Q = 0.05;
    ADC1_IN1_KalmanFilter.R = 1.0;
}

float KalmanFilter(KalmanFilter_TypeDef *KalmanFilter,float input)
{
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     KalmanFilter->Now_P = KalmanFilter->LastP + KalmanFilter->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     KalmanFilter->Kg = KalmanFilter->Now_P / (KalmanFilter->Now_P + KalmanFilter->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     KalmanFilter->out = KalmanFilter->out + KalmanFilter->Kg * (input -KalmanFilter->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 KalmanFilter->LastP 威下一次运算准备。
     KalmanFilter->LastP = (1-KalmanFilter->Kg) * KalmanFilter->Now_P;
     return KalmanFilter->out;
}
