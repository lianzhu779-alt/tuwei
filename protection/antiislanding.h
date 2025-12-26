/******************************************************************************
 * @file    antiislanding.h
 * @brief   防孤岛保护接口定义
 * @details 提供并网逆变器防孤岛检测功能，符合GB/T 19939和IEEE 1547标准
 ******************************************************************************/

#ifndef ANTIISLANDING_H
#define ANTIISLANDING_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 接口函数声明
 ******************************************************************************/

/**
 * @brief 防孤岛保护初始化
 */
void AntiIslanding_Init(void);

/**
 * @brief 使能防孤岛保护
 * @details 在并网后调用，启动孤岛检测算法
 */
void AntiIslanding_Enable(void);

/**
 * @brief 禁用防孤岛保护
 * @details 在脱网后调用，停止孤岛检测
 */
void AntiIslanding_Disable(void);

/**
 * @brief 检测是否发现孤岛
 * @return true-检测到孤岛, false-未检测到
 */
bool AntiIslanding_IsIslandDetected(void);

/**
 * @brief 防孤岛检测算法更新（100Hz调用）
 * @details 执行主动频率偏移（AFD）或其他检测算法
 */
void AntiIslanding_Update_100Hz(void);

/**
 * @brief 重置孤岛检测状态
 */
void AntiIslanding_Reset(void);

/**
 * @brief 获取孤岛检测次数统计
 * @return 孤岛检测次数
 */
uint32_t AntiIslanding_GetDetectCount(void);

#endif /* ANTIISLANDING_H */
