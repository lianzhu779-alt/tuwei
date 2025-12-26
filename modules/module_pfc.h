/******************************************************************************
 * @file    module_pfc.h
 * @brief   PFC模块接口定义（仅充电模式）
 * @details 提供PFC升压功能，将AC220V整流后升压到800V直流母线
 ******************************************************************************/

#ifndef MODULE_PFC_H
#define MODULE_PFC_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 接口函数声明
 ******************************************************************************/

/**
 * @brief PFC模块初始化
 * @return true-成功, false-失败
 */
bool PFC_Init(void);

/**
 * @brief 启动PFC
 * @return true-启动成功, false-启动失败
 */
bool PFC_Start(void);

/**
 * @brief 停止PFC
 */
void PFC_Stop(void);

/**
 * @brief 检查PFC是否运行中
 * @return true-运行中, false-已停止
 */
bool PFC_IsRunning(void);

/**
 * @brief 检查PFC软启动是否完成
 * @return true-完成, false-未完成
 */
bool PFC_IsSoftStartComplete(void);

/**
 * @brief 检查PFC输出是否稳定（800V）
 * @return true-稳定, false-不稳定
 */
bool PFC_IsOutputStable(void);

/**
 * @brief 获取母线电压
 * @return 母线电压值（V）
 */
float PFC_GetBusVoltage(void);

/**
 * @brief 获取电网电压
 * @return 电网电压有效值（V）
 */
float PFC_GetGridVoltage(void);

/**
 * @brief 获取电网电流
 * @return 电网电流有效值（A）
 */
float PFC_GetGridCurrent(void);

/**
 * @brief 获取功率因数
 * @return 功率因数（0-1）
 */
float PFC_GetPowerFactor(void);

#endif /* MODULE_PFC_H */
