#ifndef __KalmanFilter_H
#define __KalmanFilter_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
  * @brief  Structure definition of KalmanFilter
  * @note   The setting of these parameters with function KalmanFilter() is used to filter.
*/
typedef struct
{
    float LastP;        //上次估算协方差 初始化值为0.02
    float Now_P;        //当前估算协方差 初始化值为0
    float out;          //卡尔曼滤波器输出 初始化值为0
    float Kg;           //卡尔曼增益 初始化值为0
    float Q;            //过程噪声协方差 初始化值为0.001
    float R;            //观测噪声协方差 初始化值为0.543
} KalmanFilter_TypeDef; //Kalman Filter parameter

extern KalmanFilter_TypeDef ADC1_IN1_KalmanFilter;

void ADC_KalmanFilter_Init(void);
float KalmanFilter(KalmanFilter_TypeDef *KalmanFilter,float input);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__KalmanFilter_H*/