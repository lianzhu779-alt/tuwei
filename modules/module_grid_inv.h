/******************************************************************************
 * @file    module_grid_inv.h
 * @brief   并网逆变器模块接口定义（仅放电模式）
 * @details 提供800V直流到三相并网的逆变功能，带三相PLL锁相
 ******************************************************************************/

#ifndef MODULE_GRID_INV_H
#define MODULE_GRID_INV_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 类型定义
 ******************************************************************************/

/**
 * @brief 电网状态
 */
typedef enum {
    GRID_STATUS_NORMAL = 0,     /**< 电网正常 */
    GRID_STATUS_UNDERVOLTAGE,   /**< 电网欠压 */
    GRID_STATUS_OVERVOLTAGE,    /**< 电网过压 */
    GRID_STATUS_FREQ_LOW,       /**< 频率过低 */
    GRID_STATUS_FREQ_HIGH,      /**< 频率过高 */
    GRID_STATUS_PHASE_LOSS,     /**< 缺相 */
    GRID_STATUS_ABNORMAL        /**< 其他异常 */
} GridStatus_t;

/******************************************************************************
 * 接口函数声明
 ******************************************************************************/

/**
 * @brief 逆变器模块初始化
 * @return true-成功, false-失败
 */
bool GridInv_Init(void);

/**
 * @brief 启动逆变器（含PLL同步）
 * @return true-启动成功, false-启动失败
 */
bool GridInv_Start(void);

/**
 * @brief 停止逆变器
 */
void GridInv_Stop(void);

/**
 * @brief 检查逆变器是否运行中
 * @return true-运行中, false-已停止
 */
bool GridInv_IsRunning(void);

/**
 * @brief 检查电网PLL是否锁定
 * @return true-已锁定, false-未锁定
 */
bool GridInv_IsGridSynced(void);

/**
 * @brief 检查PLL是否稳定锁定（别名）
 * @return true-已锁定, false-未锁定
 */
bool GridInv_IsPLLLocked(void);

/**
 * @brief 获取电网状态
 * @return 电网状态枚举
 */
GridStatus_t GridInv_GetGridStatus(void);

/**
 * @brief 获取电网频率
 * @return 电网频率（Hz）
 */
float GridInv_GetGridFrequency(void);

/**
 * @brief 获取电网电压
 * @return 电网电压（V RMS）
 */
float GridInv_GetGridVoltage(void);

/**
 * @brief 获取相位差
 * @return 相位差（度）
 */
float GridInv_GetPhaseDifference(void);

/**
 * @brief 设置有功功率参考值
 * @param p_ref 有功功率（W）
 */
void GridInv_SetActivePower(float p_ref);

/**
 * @brief 设置无功功率参考值
 * @param q_ref 无功功率（Var）
 */
void GridInv_SetReactivePower(float q_ref);

/**
 * @brief 设置功率爬坡速率
 * @param rate 爬坡速率（W/s）
 */
void GridInv_SetPowerRampRate(float rate);

/**
 * @brief 获取实际有功功率
 * @return 有功功率（W）
 */
float GridInv_GetActivePower(void);

/**
 * @brief 获取实际无功功率
 * @return 无功功率（Var）
 */
float GridInv_GetReactivePower(void);

/**
 * @brief 获取输出电流
 * @return 输出电流（A RMS）
 */
float GridInv_GetOutputCurrent(void);

/**
 * @brief 获取直流母线电压
 * @return 直流母线电压（V）
 */
float GridInv_GetDCBusVoltage(void);

#endif /* MODULE_GRID_INV_H */
