/******************************************************************************
 * @file    grid_monitor.h
 * @brief   电网监测接口定义
 * @details 提供电网状态检测功能（电压、频率、相序等）
 ******************************************************************************/

#ifndef GRID_MONITOR_H
#define GRID_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 接口函数声明
 ******************************************************************************/

/**
 * @brief 检查电网是否正常
 * @details 综合检查电压、频率、相序等参数
 * @return true-电网正常, false-电网异常
 */
bool GridMonitor_IsGridOK(void);

/**
 * @brief 获取电网电压（V）
 * @return 电网电压有效值
 */
float GridMonitor_GetVoltage(void);

/**
 * @brief 获取电网电压（V）- 别名
 * @return 电网电压有效值
 */
float GridMonitor_GetGridVoltage(void);

/**
 * @brief 获取电网频率（Hz）
 * @return 电网频率
 */
float GridMonitor_GetFrequency(void);

/**
 * @brief 获取电网频率（Hz）- 别名
 * @return 电网频率
 */
float GridMonitor_GetGridFrequency(void);

/**
 * @brief 检查电网电压是否正常
 * @return true-正常, false-异常
 */
bool GridMonitor_IsGridVoltageNormal(void);

/**
 * @brief 检查电网频率是否正常
 * @return true-正常, false-异常
 */
bool GridMonitor_IsGridFrequencyNormal(void);

/******************************************************************************
 * 电网故障类型定义
 ******************************************************************************/

/**
 * @brief 电网故障类型枚举
 */
typedef enum {
    GRID_FAULT_NONE = 0,            /**< 无故障 */
    GRID_FAULT_OVERVOLTAGE,         /**< 电网过压 */
    GRID_FAULT_UNDERVOLTAGE,        /**< 电网欠压 */
    GRID_FAULT_OVERFREQUENCY,       /**< 频率过高 */
    GRID_FAULT_UNDERFREQUENCY,      /**< 频率过低 */
    GRID_FAULT_PHASE_LOSS           /**< 缺相 */
} GridFault_t;

/**
 * @brief 获取电网故障类型
 * @return 电网故障类型枚举
 */
GridFault_t GridMonitor_GetFaultType(void);

#endif /* GRID_MONITOR_H */
