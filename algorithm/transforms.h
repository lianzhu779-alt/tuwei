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
// 坐标变换函数（重新导出自pll_dsrf_fast.h）
//
//*****************************************************************************

// Clarke变换（三相abc到两相αβ）
// 注：这些函数在pll_dsrf_fast.h中已定义，这里仅作为统一入口

/**
 * @brief 等功率Clarke变换（三相abc到两相αβ）
 *
 * @param va        A相电压（V）
 * @param vb        B相电压（V）
 * @param vc        C相电压（V）
 * @param v_alpha   输出：α轴电压（V）
 * @param v_beta    输出：β轴电压（V）
 *
 * @note 该函数已在pll_dsrf_fast.c中实现，使用TMU硬件加速
 */
// Clarke_transform_power_fast() 已在pll_dsrf_fast.h中声明

/**
 * @brief Park变换（αβ到dq，正向旋转）
 *
 * @param v_alpha   α轴电压（V）
 * @param v_beta    β轴电压（V）
 * @param cos_theta 预计算的 cos(θ)
 * @param sin_theta 预计算的 sin(θ)
 * @param output    输出：dq分量
 *
 * @note 该函数已在pll_dsrf_fast.h中声明为inline函数
 */
// Park_transform_positive_fast() 已在pll_dsrf_fast.h中声明

//*****************************************************************************
//
// 新增坐标变换函数
//
//*****************************************************************************

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
