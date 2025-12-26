/******************************************************************************
 * @file    module_llc_bidir.h
 * @brief   双向LLC模块接口定义（双交错IPOS结构）
 * @details 支持正向降压（充电）和反向升压（放电）两种模式
 ******************************************************************************/

#ifndef MODULE_LLC_BIDIR_H
#define MODULE_LLC_BIDIR_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 类型定义
 ******************************************************************************/

/**
 * @brief LLC工作方向
 */
typedef enum {
    LLC_DIR_FORWARD = 0,    /**< 正向：800V→125V（降压/充电） */
    LLC_DIR_REVERSE = 1     /**< 反向：125V→800V（升压/放电） */
} LLC_Direction_t;

/**
 * @brief LLC控制模式
 */
typedef enum {
    LLC_MODE_VOLTAGE = 0,   /**< 恒压模式 */
    LLC_MODE_CURRENT,       /**< 恒流模式 */
    LLC_MODE_POWER          /**< 恒功率模式 */
} LLC_ControlMode_t;

/******************************************************************************
 * 接口函数声明
 ******************************************************************************/

/**
 * @brief LLC模块初始化
 * @return true-成功, false-失败
 */
bool LLC_Init(void);

/**
 * @brief 启动LLC
 * @param dir 工作方向（正向/反向）
 * @return true-启动成功, false-启动失败
 */
bool LLC_Start(LLC_Direction_t dir);

/**
 * @brief 停止LLC
 */
void LLC_Stop(void);

/**
 * @brief 检查LLC是否运行中
 * @return true-运行中, false-已停止
 */
bool LLC_IsRunning(void);

/**
 * @brief 检查LLC软启动是否完成
 * @return true-完成, false-未完成
 */
bool LLC_IsSoftStartComplete(void);

/**
 * @brief 检查两路LLC均压均流是否正常
 * @return true-正常, false-失衡
 */
bool LLC_IsBalanced(void);

/**
 * @brief 检查LLC输入均压是否正常（C1≈C2）
 * @return true-正常, false-失衡
 */
bool LLC_IsInputBalanced(void);

/**
 * @brief 检查LLC输出均流是否正常（Iout1≈Iout2）
 * @return true-正常, false-失衡
 */
bool LLC_IsOutputBalanced(void);

/**
 * @brief 获取输入电压不平衡量
 * @return |Vc1-Vc2|（V）
 */
float LLC_GetInputImbalance(void);

/**
 * @brief 获取输出电流不平衡量
 * @return |Iout1-Iout2|（A）
 */
float LLC_GetOutputImbalance(void);

/**
 * @brief 获取LLC输入电压1（C1电压）
 * @return C1电压值（V）
 */
float LLC_GetInputVoltage1(void);

/**
 * @brief 获取LLC输入电压2（C2电压）
 * @return C2电压值（V）
 */
float LLC_GetInputVoltage2(void);

/**
 * @brief 获取LLC总输入电压
 * @details 正向模式返回800V侧，反向模式返回125V侧
 * @return 总输入电压值（V）
 */
float LLC_GetInputVoltage(void);

/**
 * @brief 获取LLC总输入电流
 * @return 总输入电流（A）
 */
float LLC_GetInputCurrent(void);

/**
 * @brief 获取LLC1输入电流
 * @return LLC1输入电流（A）
 */
float LLC_GetInputCurrent1(void);

/**
 * @brief 获取LLC2输入电流
 * @return LLC2输入电流（A）
 */
float LLC_GetInputCurrent2(void);

/**
 * @brief 获取LLC输出电压
 * @details 正向模式返回125V侧，反向模式返回800V侧
 * @return 输出电压值（V）
 */
float LLC_GetOutputVoltage(void);

/**
 * @brief 获取LLC1输出电压（仅反向模式）
 * @return LLC1输出电压值（V）
 */
float LLC_GetOutputVoltage1(void);

/**
 * @brief 获取LLC2输出电压（仅反向模式）
 * @return LLC2输出电压值（V）
 */
float LLC_GetOutputVoltage2(void);

/**
 * @brief 获取LLC总输出电流
 * @return 总输出电流（A）
 */
float LLC_GetOutputCurrent(void);

/**
 * @brief 获取LLC1输出电流
 * @return LLC1输出电流（A）
 */
float LLC_GetOutputCurrent1(void);

/**
 * @brief 获取LLC2输出电流
 * @return LLC2输出电流（A）
 */
float LLC_GetOutputCurrent2(void);

/**
 * @brief 获取LLC输出功率
 * @return 输出功率（W）
 */
float LLC_GetOutputPower(void);

/**
 * @brief 设置电压参考值
 * @param v_ref 电压参考值（V）
 */
void LLC_SetVoltageRef(float v_ref);

/**
 * @brief 设置电流参考值
 * @param i_ref 电流参考值（A）
 */
void LLC_SetCurrentRef(float i_ref);

/**
 * @brief 设置功率参考值
 * @param p_ref 功率参考值（W）
 */
void LLC_SetPowerRef(float p_ref);

/**
 * @brief 设置控制模式
 * @param mode 控制模式（电压/电流/功率）
 */
void LLC_SetControlMode(LLC_ControlMode_t mode);

#endif /* MODULE_LLC_BIDIR_H */
