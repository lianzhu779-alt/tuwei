/**
 * @file sogi_pll.h
 * @brief 双同步解耦锁相环 (DSOGI-PLL) - CLA优化版本
 * @details 本模块实现了DSOGI-PLL算法，融合了SOGI和DSRF-PLL的优点：
 *          - 二阶广义积分器(SOGI)滤波
 *          - 正交信号生成（α/β轴）
 *          - 基于PI控制器的锁相环（带Anti-windup）
 *          - 正负序分量提取
 *          - 电网频率与相位跟踪
 *          - CLA数学优化：直接使用CLAmath内联sin/cos，减少重复计算
 *
 * @note 针对TI C2000 CLA环境优化，使用CLA内联数学函数
 * @date 2025-12-23
 * @version 2.0 - 融合DSRF-PLL优化特性
 */

#ifndef SOGI_PLL_H
#define SOGI_PLL_H

#include "CLAmath.h"   // 直接使用CLA数学库API

//*****************************************************************************
//
// 常量定义
//
//*****************************************************************************

/** @brief 2π常量 */
#define TWO_PI          6.28318530718f

/** @brief π常量 */
#define PI              3.14159265359f

/** @brief √2 常量 */
#define SQRT2           1.414213562373095f

//*****************************************************************************
//
// 内联辅助函数
//
//*****************************************************************************

/**
 * @brief 饱和限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static inline float saturate_inline(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

//*****************************************************************************
//
// 数据结构定义
//
//*****************************************************************************

/**
 * @brief SOGI-PLL状态结构体
 * @details 包含SOGI滤波器和PLL的所有状态变量
 */
typedef struct
{
    // ========== SOGI参数 ==========
    float k;                /**< SOGI增益系数，典型值√2 */
    float wn;               /**< SOGI自然频率（rad/s），通常等于电网角频率 */

    // ========== SOGI α轴状态 ==========
    float x1_alpha;         /**< α轴状态变量1（同相输出） */
    float x2_alpha;         /**< α轴状态变量2（正交输出） */
    float u_alpha;          /**< α轴滤波后输出 */
    float qu_alpha;         /**< α轴正交输出（90°相移） */

    // ========== SOGI β轴状态 ==========
    float x1_beta;          /**< β轴状态变量1（同相输出） */
    float x2_beta;          /**< β轴状态变量2（正交输出） */
    float u_beta;           /**< β轴滤波后输出 */
    float qu_beta;          /**< β轴正交输出（90°相移） */

    // ========== PLL状态 ==========
    float theta;            /**< PLL锁定的相角（rad），范围[0, 2π] */
    float freq;             /**< PLL锁定的频率（Hz） */
    float omega;            /**< PLL锁定的角频率（rad/s） */

    // ========== PI控制器状态（Anti-windup） ==========
    float pll_integrator;   /**< PI控制器积分项 */
    float error_prev;       /**< 上一次误差值 */

    // ========== PLL参数 ==========
    float kp_pll;           /**< PLL比例增益 */
    float ki_pll;           /**< PLL积分增益 */
    float kb_pll;           /**< Anti-windup回算增益，典型值=Ki */
    float omega_nom;        /**< 额定角频率（rad/s），如50Hz = 314.159 */
    float freq_nom;         /**< 额定频率（Hz） */
    float dOmegaMax;        /**< 频率偏差限幅（rad/s），典型值±50 rad/s */

    // ========== 离散化参数 ==========
    float Ts;               /**< 采样周期（s） */

    // ========== 正负序分量 ==========
    float d_pos;            /**< 正序D轴分量 */
    float q_pos;            /**< 正序Q轴分量 */
    float d_neg;            /**< 负序D轴分量 */
    float q_neg;            /**< 负序Q轴分量 */

    // ========== 优化：缓存sin/cos值（减少重复计算） ==========
    float cached_sin_theta; /**< 缓存的sin(θ) */
    float cached_cos_theta; /**< 缓存的cos(θ) */

    // ========== 锁定检测 ==========
    uint16_t lockCounter;   /**< 锁定计数器 */
    uint8_t locked;         /**< 锁定标志，1=已锁定 */

} SOGI_PLL;

//*****************************************************************************
//
// 函数接口
//
//*****************************************************************************

/**
 * @brief SOGI-PLL初始化
 * @details 初始化所有状态变量和参数
 *
 * @param sogi      SOGI-PLL对象指针
 * @param fs        采样频率（Hz），如100kHz
 * @param f_nom     额定电网频率（Hz），如50Hz或60Hz
 *
 * @note 默认PLL参数：kp=100, ki=5000, kb=5000（Anti-windup）
 */
static inline void SOGI_PLL_init(SOGI_PLL *sogi, float fs, float f_nom)
{
    // 采样参数
    sogi->Ts = 1.0f / fs;
    sogi->freq_nom = f_nom;
    sogi->omega_nom = TWO_PI * f_nom;

    // SOGI参数
    sogi->k = SQRT2;                // 典型值√2，提供良好的滤波特性
    sogi->wn = sogi->omega_nom;     // 自然频率等于额定频率

    // PLL参数（经验值，可根据动态响应需求调整）
    sogi->kp_pll = 266.0f;          // 比例增益
    sogi->ki_pll = 35530.0f;         // 积分增益
    sogi->kb_pll = sogi->ki_pll;         // Anti-windup回算增益
    sogi->dOmegaMax = 50.0f;        // 频率偏差限幅 ±50 rad/s (约±8Hz)

    // 初始化状态
    sogi->x1_alpha = 0.0f;
    sogi->x2_alpha = 0.0f;
    sogi->u_alpha = 0.0f;
    sogi->qu_alpha = 0.0f;

    sogi->x1_beta = 0.0f;
    sogi->x2_beta = 0.0f;
    sogi->u_beta = 0.0f;
    sogi->qu_beta = 0.0f;

    sogi->theta = 0.0f;
    sogi->freq = f_nom;
    sogi->omega = sogi->omega_nom;
    sogi->pll_integrator = 0.0f;
    sogi->error_prev = 0.0f;

    sogi->d_pos = 0.0f;
    sogi->q_pos = 0.0f;
    sogi->d_neg = 0.0f;
    sogi->q_neg = 0.0f;

    sogi->cached_sin_theta = 0.0f;
    sogi->cached_cos_theta = 1.0f;

    sogi->lockCounter = 0;
    sogi->locked = 0;
}

/**
 * @brief SOGI-PLL重置
 * @details 重置所有状态变量到初始值，保持参数不变
 *
 * @param sogi  SOGI-PLL对象指针
 */
static inline void SOGI_PLL_reset(SOGI_PLL *sogi)
{
    sogi->x1_alpha = 0.0f;
    sogi->x2_alpha = 0.0f;
    sogi->u_alpha = 0.0f;
    sogi->qu_alpha = 0.0f;

    sogi->x1_beta = 0.0f;
    sogi->x2_beta = 0.0f;
    sogi->u_beta = 0.0f;
    sogi->qu_beta = 0.0f;

    sogi->theta = 0.0f;
    sogi->freq = sogi->freq_nom;
    sogi->omega = sogi->omega_nom;
    sogi->pll_integrator = 0.0f;
    sogi->error_prev = 0.0f;

    sogi->d_pos = 0.0f;
    sogi->q_pos = 0.0f;
    sogi->d_neg = 0.0f;
    sogi->q_neg = 0.0f;

    sogi->cached_sin_theta = 0.0f;
    sogi->cached_cos_theta = 1.0f;

    sogi->lockCounter = 0;
    sogi->locked = 0;
}

/**
 * @brief SOGI-PLL单步运行（CLA优化版本 v2.0）
 * @details 执行一次SOGI滤波和PLL更新，融合了DSRF-PLL的优化特性：
 *          1. SOGI α/β轴滤波
 *          2. 使用CLAsincos_inline一次性计算sin/cos（CLA优化）
 *          3. 缓存sin/cos值，避免重复计算
 *          4. PLL相位检测与频率跟踪（带Anti-windup）
 *          5. 正负序分量计算
 *
 * @param sogi      SOGI-PLL对象指针
 * @param v_alpha   输入α轴电压
 * @param v_beta    输入β轴电压
 *
 * @note 使用前向欧拉法离散化，适合高频采样（≥10kHz）
 * @note v2.0优化：切换为纯CLA数学API，保留Anti-windup控制
 */
static inline void SOGI_PLL_run(SOGI_PLL *sogi, float v_alpha, float v_beta)
{
    float k_wn;
    float dx1_alpha, dx2_alpha, dx1_beta, dx2_beta;
    float phase_error;
    float upre, usat;
    float seq_alpha, seq_beta;
    float sin_theta, cos_theta;

    // ========== SOGI滤波器（α轴和β轴） ==========
    k_wn = sogi->k * sogi->wn;

    // α轴状态方程
    dx1_alpha = k_wn * (v_alpha - sogi->x1_alpha) - sogi->omega * sogi->x2_alpha;
    dx2_alpha = sogi->omega * sogi->x1_alpha;

    sogi->x1_alpha += dx1_alpha * sogi->Ts;
    sogi->x2_alpha += dx2_alpha * sogi->Ts;

    sogi->u_alpha = sogi->x1_alpha;      // 同相输出
    sogi->qu_alpha = sogi->x2_alpha;     // 正交输出（90°滞后）

    // β轴状态方程
    dx1_beta = k_wn * (v_beta - sogi->x1_beta) - sogi->omega * sogi->x2_beta;
    dx2_beta = sogi->omega * sogi->x1_beta;

    sogi->x1_beta += dx1_beta * sogi->Ts;
    sogi->x2_beta += dx2_beta * sogi->Ts;

    sogi->u_beta = sogi->x1_beta;
    sogi->qu_beta = sogi->x2_beta;

    // ========== 优化：一次性计算sin/cos并缓存 ==========
    CLAsincos_inline(sogi->theta, &sogi->cached_sin_theta, &sogi->cached_cos_theta);
    sin_theta = sogi->cached_sin_theta;
    cos_theta = sogi->cached_cos_theta;

    // ========== 正负序分量提取（使用缓存的sin/cos） ==========
    // 先计算正序
    seq_alpha = sogi->u_alpha - sogi->qu_beta;
    seq_beta  = sogi->u_beta  + sogi->qu_alpha;
    sogi->d_pos = 0.5f * (seq_alpha * cos_theta + seq_beta * sin_theta);
    sogi->q_pos = 0.5f * (-seq_alpha * sin_theta + seq_beta * cos_theta);

    // 再复用变量计算负序
    seq_alpha = sogi->u_alpha + sogi->qu_beta;
    seq_beta  = sogi->u_beta  - sogi->qu_alpha;
    sogi->d_neg = 0.5f * (seq_alpha * cos_theta - seq_beta * sin_theta);
    sogi->q_neg = 0.5f * (seq_alpha * sin_theta + seq_beta * cos_theta);

    // ========== PLL相位检测（带浮动归一化） ==========
    // 归一化正序Q轴，保证PLL增益不受电压幅值影响
    float mag_sq = sogi->d_pos * sogi->d_pos + sogi->q_pos * sogi->q_pos;
    if (mag_sq < 400.0f)
    {
        mag_sq = 400.0f; // 下限保护防止除零
    }
    float inv_mag = CLAisqrt_inline(mag_sq);
    phase_error = sogi->q_pos * inv_mag;

    // ========== Anti-windup PI控制器（回算式） ==========
    // 参考DSRF-PLL实现
    upre = sogi->kp_pll * phase_error + sogi->pll_integrator;
    usat = saturate_inline(upre, -sogi->dOmegaMax, sogi->dOmegaMax);

    // 积分器更新（包含Anti-windup回算项）
    sogi->pll_integrator += (sogi->ki_pll * phase_error +
                             sogi->kb_pll * (usat - upre)) * sogi->Ts;

    // 频率偏差
    float dOmega = usat;

    // ========== 更新频率和相位 ==========
    sogi->omega = sogi->omega_nom + dOmega;
    sogi->freq = sogi->omega / TWO_PI;

    // 相位积分
    sogi->theta += sogi->omega * sogi->Ts;

    // 相位归一化到 [0, 2π]
    if (sogi->theta >= TWO_PI) {
        sogi->theta -= TWO_PI;
    } else if (sogi->theta < 0.0f) {
        sogi->theta += TWO_PI;
    }

    // ========== 锁定检测 ==========
    float freq_error = sogi->freq - sogi->freq_nom;
    if (freq_error < 0.0f) freq_error = -freq_error;

    if (freq_error < 0.5f) {  // 容差0.5Hz
        sogi->lockCounter++;
        if (sogi->lockCounter >= 100) {  // 100个周期后认为锁定
            sogi->locked = 1;
            sogi->lockCounter = 100;
        }
    } else {
        sogi->lockCounter = 0;
        sogi->locked = 0;
    }
}

/**
 * @brief 设置PLL增益参数（包含Anti-windup）
 * @details 允许在运行时调整PLL的动态响应特性
 *
 * @param sogi  SOGI-PLL对象指针
 * @param kp    比例增益
 * @param ki    积分增益
 * @param kb    Anti-windup回算增益（通常=ki）
 *
 * @note 增益过大可能导致振荡，过小会降低响应速度
 */
static inline void SOGI_PLL_set_gains(SOGI_PLL *sogi, float kp, float ki, float kb)
{
    sogi->kp_pll = kp;
    sogi->ki_pll = ki;
    sogi->kb_pll = kb;
}

/**
 * @brief 设置SOGI滤波器参数
 * @details 调整SOGI的增益和自然频率
 *
 * @param sogi  SOGI-PLL对象指针
 * @param k     增益系数，典型值√2
 * @param wn    自然频率（rad/s）
 */
static inline void SOGI_PLL_set_sogi_params(SOGI_PLL *sogi, float k, float wn)
{
    sogi->k = k;
    sogi->wn = wn;
}

/**
 * @brief 获取PLL锁定状态
 * @details 判断PLL是否已稳定锁定到电网频率
 *
 * @param sogi          SOGI-PLL对象指针
 * @param freq_tolerance 频率容差（Hz），典型值0.5Hz
 * @return 1表示已锁定，0表示未锁定
 */
static inline int SOGI_PLL_is_locked(SOGI_PLL *sogi, float freq_tolerance)
{
    return sogi->locked;
}

/**
 * @brief 获取缓存的sin(θ)值（优化：避免重复计算）
 */
static inline float SOGI_PLL_get_sin_theta(SOGI_PLL *sogi)
{
    return sogi->cached_sin_theta;
}

/**
 * @brief 获取缓存的cos(θ)值（优化：避免重复计算）
 */
static inline float SOGI_PLL_get_cos_theta(SOGI_PLL *sogi)
{
    return sogi->cached_cos_theta;
}

#endif // SOGI_PLL_H

//
// End of File
//
