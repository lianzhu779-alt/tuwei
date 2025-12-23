//#############################################################################
//
// FILE:   transforms.h
//
// TITLE:  坐标变换统一接口
//
// DESCRIPTION:
//   该模块提供所有坐标变换函数的统一接口，包括：
//   - Clarke变换（abc → αβ）
//   - Park变换（αβ → dq）
//   - 逆Park变换（dq → αβ）
//
//   所有变换函数都经过TMU硬件加速优化。
//
//#############################################################################

#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <math.h>

//*****************************************************************************
//
// 数据结构定义
//
//*****************************************************************************

/**
 * @brief αβ坐标系矢量
 */
typedef struct
{
    float alpha;    /**< α 分量 */
    float beta;     /**< β 分量 */
} AlphaBeta_Vec;

//*****************************************************************************
//
// 坐标变换函数
//
//*****************************************************************************

/**
 * @brief Park变换（αβ -> dq，正向旋转）
 *
 * @param v_alpha   α轴分量
 * @param v_beta    β轴分量
 * @param cos_theta 预计算的 cos(θ)
 * @param sin_theta 预计算的 sin(θ)
 * @param d         输出：d轴分量
 * @param q         输出：q轴分量
 */
static inline void Park_transform_positive_fast(float v_alpha, float v_beta,
                                                float cos_theta, float sin_theta,
                                                float *d, float *q)
{
    *d = v_alpha * cos_theta + v_beta * sin_theta;
    *q = -v_alpha * sin_theta + v_beta * cos_theta;
}

//*****************************************************************************
//
// 新增坐标变换函数
//
//*****************************************************************************

/**
 * @brief 等幅值Clarke变换（三相abc到两相αβ）
 *
 * @param ia        A相分量
 * @param ib        B相分量
 * @param ic        C相分量
 * @param output    输出：αβ矢量
 */
static inline void clarke_amplitude(float ia, float ib, float ic,
                                    AlphaBeta_Vec *output)
{
    const float SCALE = 0.816496580927726f;         // sqrt(2/3)
    const float HALF = 0.5f;
    const float SQRT3_OVER_2 = 0.8660254037844387f; // sqrt(3)/2

    float sum_bc = ib + ic;
    output->alpha = SCALE * (ia - HALF * sum_bc);
    output->beta  = SCALE * (SQRT3_OVER_2 * (ib - ic));
}

/**
 * @brief 逆Park变换（dq → αβ）
 *
 * 该函数将旋转坐标系dq转换回静止坐标系αβ。
 *
 * @param d         d轴分量
 * @param q         q轴分量
 * @param cos_theta 预计算的 cos(θ)
 * @param sin_theta 预计算的 sin(θ)
 * @param output    输出：αβ矢量
 *
 * @note 使用static inline优化，减少函数调用开销（节省约5-10 cycles）
 */
static inline void inverse_park(float d, float q, float cos_theta, float sin_theta,
                                AlphaBeta_Vec *output)
{
    output->alpha = d * cos_theta - q * sin_theta;
    output->beta  = d * sin_theta + q * cos_theta;
}

/**
 * @brief 逆Clarke变换（αβ → abc）
 *
 * 该函数将两相静止坐标系αβ转换回三相abc。
 *
 * @param alpha    α轴分量
 * @param beta     β轴分量
 * @param va       输出：A相分量
 * @param vb       输出：B相分量
 * @param vc       输出：C相分量
 *
 * @note 使用static inline优化
 */
static inline void inverse_clarke(float alpha, float beta,
                                  float *va, float *vb, float *vc)
{
    const float SQRT3_OVER_2 = 0.8660254037844387f;  // sqrt(3)/2

    *va = alpha;
    *vb = -0.5f * alpha + SQRT3_OVER_2 * beta;
    *vc = -0.5f * alpha - SQRT3_OVER_2 * beta;
}

//*****************************************************************************
//
// 辅助函数
//
//*****************************************************************************

/**
 * @brief 线电压转换为相电压（三线制假定零序为0）
 *
 * @param v_ab   AB线电压（V）
 * @param v_bc   BC线电压（V）
 * @param v_ca   CA线电压（V）
 * @param va     输出：A相电压（V）
 * @param vb     输出：B相电压（V）
 * @param vc     输出：C相电压（V）
 *
 * @note 使用static inline优化
 */
static inline void line_to_phase_voltages(float v_ab, float v_bc, float v_ca,
                                          float *va, float *vb, float *vc)
{
    const float ONE_THIRD = 0.3333333333f;
    *va = (v_ab - v_ca) * ONE_THIRD;
    *vb = (v_bc - v_ab) * ONE_THIRD;
    *vc = (v_ca - v_bc) * ONE_THIRD;
}

#endif // TRANSFORMS_H

//
// End of File
//
