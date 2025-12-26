/******************************************************************************
 * @file    charging_control.c
 * @brief   充电模式管理模块实现
 * @details 实现三段式充电控制（CC-CV-Float），包括启动序列和保护功能
 * @author  Generated for TI DSP280049 Platform
 * @date    2025-12-26
 ******************************************************************************/

#include "charging_control.h"
#include "module_pfc.h"
#include "module_llc_bidir.h"
#include "grid_monitor.h"
#include "hal.h"
#include <string.h>
#include <math.h>

/******************************************************************************
 * 全局静态变量
 ******************************************************************************/

/**
 * @brief 充电状态全局变量
 */
static ChargeStatus_t g_charge_status = {.stage = CHARGE_STAGE_IDLE};

/**
 * @brief 充电配置全局变量
 */
static ChargeConfig_t g_charge_config = {0};

/**
 * @brief 模块初始化标志
 */
static bool g_initialized = false;

/**
 * @brief 上一次100Hz任务的时间戳（用于积分计算）
 */
static uint32_t g_last_100hz_time = 0;

/******************************************************************************
 * 内部辅助函数声明
 ******************************************************************************/

/* 阶段处理函数 */
static void Stage_Precharge_Handler(void);
static void Stage_PFCStart_Handler(void);
static void Stage_LLCStart_Handler(void);
static void Stage_CC_Handler(void);
static void Stage_CV_Handler(void);
static void Stage_Float_Handler(void);

/* 保护和检查函数 */
static void Protection_Check(void);
static void Update_Measurements(void);
static void Update_LLC_Balance_Status(void);

/* 状态转换和辅助函数 */
static void TransitionTo(ChargeStage_t new_stage);
static uint32_t GetElapsedTime(uint32_t start_time);
static void SetDefaultConfig(void);
static void ResetStatus(void);

/******************************************************************************
 * 公共接口函数实现
 ******************************************************************************/

/**
 * @brief 充电模块初始化
 */
void Charging_Init(void)
{
    // 初始化状态结构体
    memset(&g_charge_status, 0, sizeof(ChargeStatus_t));
    g_charge_status.stage = CHARGE_STAGE_IDLE;
    g_charge_status.is_running = false;

    // 设置默认配置
    SetDefaultConfig();

    // 初始化时间戳
    g_last_100hz_time = HAL_GetTick();

    g_initialized = true;
}

/**
 * @brief 启动充电
 */
bool Charging_Start(const ChargeConfig_t* config)
{
    // 检查初始化状态
    if (!g_initialized) {
        return false;
    }

    // 检查是否已经在运行
    if (g_charge_status.is_running) {
        return false;
    }

    // 检查电网状态
    if (!GridMonitor_IsGridOK()) {
        g_charge_status.fault_flags = CHARGE_FAULT_GRID_FAIL;
        g_charge_status.stage = CHARGE_STAGE_ERROR;
        return false;
    }

    // 应用配置（如果提供）
    if (config != NULL) {
        memcpy(&g_charge_config, config, sizeof(ChargeConfig_t));
    } else {
        SetDefaultConfig();
    }

    // 重置状态
    ResetStatus();

    // 设置初始状态
    g_charge_status.stage = CHARGE_STAGE_PRECHARGE;
    g_charge_status.is_running = true;
    g_charge_status.stage_entry_time = HAL_GetTick();
    g_charge_status.charge_start_time = HAL_GetTick();
    g_last_100hz_time = HAL_GetTick();

    return true;
}

/**
 * @brief 正常停止充电
 */
void Charging_Stop(void)
{
    if (!g_charge_status.is_running) {
        return;
    }

    // 按顺序停止模块
    // 1. 停止LLC（降压输出）
    if (LLC_IsRunning()) {
        LLC_Stop();
        HAL_Delay(100);  // 等待LLC完全停止
    }

    // 2. 停止PFC
    if (PFC_IsRunning()) {
        PFC_Stop();
        HAL_Delay(100);  // 等待PFC完全停止
    }

    // 3. 断开继电器
    HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);

    // 4. 重置状态
    g_charge_status.is_running = false;
    g_charge_status.stage = CHARGE_STAGE_IDLE;
}

/**
 * @brief 紧急停止充电
 */
void Charging_Abort(void)
{
    // 立即停止所有模块（不等待）
    LLC_Stop();
    PFC_Stop();

    // 立即断开所有继电器
    HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);

    // 重置状态
    g_charge_status.is_running = false;
    g_charge_status.stage = CHARGE_STAGE_ERROR;
}

/**
 * @brief 充电控制任务 - 1kHz调用
 */
void Charging_Run_1kHz(void)
{
    // 检查运行状态
    if (!g_charge_status.is_running) {
        return;
    }

    // 更新测量数据
    Update_Measurements();

    // 更新LLC均压均流状态
    Update_LLC_Balance_Status();

    // 执行保护检查
    Protection_Check();

    // 如果发生故障，停止充电
    if (g_charge_status.stage == CHARGE_STAGE_ERROR) {
        Charging_Abort();
        return;
    }

    // 阶段状态机
    switch (g_charge_status.stage) {
        case CHARGE_STAGE_PRECHARGE:
            Stage_Precharge_Handler();
            break;

        case CHARGE_STAGE_PFC_START:
            Stage_PFCStart_Handler();
            break;

        case CHARGE_STAGE_LLC_START:
            Stage_LLCStart_Handler();
            break;

        case CHARGE_STAGE_CC:
            Stage_CC_Handler();
            break;

        case CHARGE_STAGE_CV:
            Stage_CV_Handler();
            break;

        case CHARGE_STAGE_FLOAT:
            Stage_Float_Handler();
            break;

        case CHARGE_STAGE_COMPLETE:
            // 充电完成，保持当前状态
            // 可选：自动切换到浮充维护
            break;

        case CHARGE_STAGE_ERROR:
            // 错误状态，等待外部处理
            Charging_Abort();
            break;

        default:
            // 未知状态，强制停止
            g_charge_status.fault_flags |= CHARGE_FAULT_LLC_FAIL;
            TransitionTo(CHARGE_STAGE_ERROR);
            break;
    }
}

/**
 * @brief 充电统计任务 - 100Hz调用
 */
void Charging_Run_100Hz(void)
{
    if (!g_charge_status.is_running) {
        return;
    }

    // 只在实际充电阶段进行统计（CC/CV/Float）
    if (g_charge_status.stage < CHARGE_STAGE_CC) {
        return;
    }

    // 计算时间间隔（秒）
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - g_last_100hz_time) / 1000.0f;  // 转换为秒
    g_last_100hz_time = current_time;

    // 更新充电时间
    g_charge_status.charge_time_sec =
        (HAL_GetTick() - g_charge_status.charge_start_time) / 1000;

    // 累积充电电量（Ah）- 使用梯形积分
    // Q = ∫I·dt，这里近似为 I × dt
    g_charge_status.charge_ah += g_charge_status.battery_current * dt / 3600.0f;

    // 累积充电能量（Wh）- 使用梯形积分
    // E = ∫P·dt = ∫V·I·dt
    g_charge_status.charge_wh += g_charge_status.battery_power * dt / 3600.0f;

    // 估算SOC（基于充入电量）
    if (g_charge_config.battery_capacity_ah > 0) {
        float soc_increment = (g_charge_status.charge_ah /
                               g_charge_config.battery_capacity_ah) * 100.0f;
        // 假设初始SOC为0%（可以根据初始电压估算）
        g_charge_status.soc_estimate = soc_increment;

        // 限制在0-100%
        if (g_charge_status.soc_estimate > 100.0f) {
            g_charge_status.soc_estimate = 100.0f;
        }
    }
}

/**
 * @brief 检查充电是否运行中
 */
bool Charging_IsRunning(void)
{
    return g_charge_status.is_running;
}

/**
 * @brief 获取当前充电阶段
 */
ChargeStage_t Charging_GetStage(void)
{
    return g_charge_status.stage;
}

/**
 * @brief 获取充电状态指针
 */
const ChargeStatus_t* Charging_GetStatus(void)
{
    return &g_charge_status;
}

/**
 * @brief 获取充电进度
 */
float Charging_GetProgress(void)
{
    // 基于SOC估算返回进度
    if (g_charge_status.soc_estimate > 0) {
        return g_charge_status.soc_estimate;
    }

    // 如果没有SOC，基于阶段返回粗略进度
    switch (g_charge_status.stage) {
        case CHARGE_STAGE_IDLE:
        case CHARGE_STAGE_PRECHARGE:
        case CHARGE_STAGE_PFC_START:
        case CHARGE_STAGE_LLC_START:
            return 0.0f;

        case CHARGE_STAGE_CC:
            return 30.0f;  // CC阶段假设30%

        case CHARGE_STAGE_CV:
            return 70.0f;  // CV阶段假设70%

        case CHARGE_STAGE_FLOAT:
            return 95.0f;  // Float阶段假设95%

        case CHARGE_STAGE_COMPLETE:
            return 100.0f;

        default:
            return 0.0f;
    }
}

/**
 * @brief 设置充电电流限制
 */
void Charging_SetCurrentLimit(float i_max)
{
    if (i_max > 0 && i_max <= DEFAULT_PROTECTION_CURRENT_MAX) {
        g_charge_config.charge_current_cc = i_max;

        // 如果正在CC阶段，立即更新参考值
        if (g_charge_status.stage == CHARGE_STAGE_CC && LLC_IsRunning()) {
            LLC_SetCurrentRef(i_max);
        }
    }
}

/**
 * @brief 设置充电电压限制
 */
void Charging_SetVoltageLimit(float v_max)
{
    if (v_max > 0 && v_max <= DEFAULT_PROTECTION_VOLTAGE_MAX) {
        g_charge_config.charge_voltage_cv = v_max;

        // 如果正在CV阶段，立即更新参考值
        if (g_charge_status.stage == CHARGE_STAGE_CV && LLC_IsRunning()) {
            LLC_SetVoltageRef(v_max);
        }
    }
}

/**
 * @brief 设置充电功率限制
 */
void Charging_SetPowerLimit(float p_max)
{
    if (p_max > 0) {
        g_charge_config.charge_power_max = p_max;
    }
}

/**
 * @brief 清除故障标志
 */
bool Charging_ClearFaults(void)
{
    // 只有在停止状态下才能清除故障
    if (g_charge_status.is_running) {
        return false;
    }

    g_charge_status.fault_flags = CHARGE_FAULT_NONE;
    g_charge_status.warning_flags = CHARGE_WARN_NONE;
    g_charge_status.stage = CHARGE_STAGE_IDLE;

    return true;
}

/**
 * @brief 获取充电阶段名称字符串
 */
const char* Charging_GetStageName(ChargeStage_t stage)
{
    switch (stage) {
        case CHARGE_STAGE_IDLE:       return "IDLE";
        case CHARGE_STAGE_PRECHARGE:  return "PRECHARGE";
        case CHARGE_STAGE_PFC_START:  return "PFC_START";
        case CHARGE_STAGE_LLC_START:  return "LLC_START";
        case CHARGE_STAGE_CC:         return "CC";
        case CHARGE_STAGE_CV:         return "CV";
        case CHARGE_STAGE_FLOAT:      return "FLOAT";
        case CHARGE_STAGE_COMPLETE:   return "COMPLETE";
        case CHARGE_STAGE_ERROR:      return "ERROR";
        default:                      return "UNKNOWN";
    }
}

/**
 * @brief 获取故障描述字符串
 */
const char* Charging_GetFaultString(uint32_t fault_code)
{
    if (fault_code == CHARGE_FAULT_NONE) return "No Fault";
    if (fault_code & CHARGE_FAULT_GRID_FAIL) return "Grid Fail";
    if (fault_code & CHARGE_FAULT_PRECHARGE_TO) return "Precharge Timeout";
    if (fault_code & CHARGE_FAULT_PFC_START_TO) return "PFC Start Timeout";
    if (fault_code & CHARGE_FAULT_LLC_START_TO) return "LLC Start Timeout";
    if (fault_code & CHARGE_FAULT_IMBALANCE_V) return "Voltage Imbalance";
    if (fault_code & CHARGE_FAULT_IMBALANCE_I) return "Current Imbalance";
    if (fault_code & CHARGE_FAULT_BATTERY_OV) return "Battery Overvoltage";
    if (fault_code & CHARGE_FAULT_BATTERY_OC) return "Battery Overcurrent";
    if (fault_code & CHARGE_FAULT_CHARGE_TIMEOUT) return "Charge Timeout";
    if (fault_code & CHARGE_FAULT_PFC_UNSTABLE) return "PFC Unstable";
    if (fault_code & CHARGE_FAULT_BUS_OV) return "Bus Overvoltage";
    if (fault_code & CHARGE_FAULT_BUS_UV) return "Bus Undervoltage";
    if (fault_code & CHARGE_FAULT_LLC_FAIL) return "LLC Fail";
    if (fault_code & CHARGE_FAULT_PFC_FAIL) return "PFC Fail";

    return "Multiple Faults";
}

/******************************************************************************
 * 阶段处理函数实现
 ******************************************************************************/

/**
 * @brief 预充电阶段处理
 * @details 通过预充电继电器对800V母线进行预充电
 *          目标：母线电压达到350V以上
 *          超时：5秒
 */
static void Stage_Precharge_Handler(void)
{
    static bool precharge_started = false;

    // 首次进入：启动预充电
    if (!precharge_started) {
        // 确保主继电器断开
        HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
        HAL_Delay(10);

        // 接通预充电继电器
        HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_SET);
        precharge_started = true;
        return;
    }

    // 读取母线电压（通过PFC模块接口）
    float vbus = PFC_GetBusVoltage();
    g_charge_status.bus_voltage = vbus;

    // 检查预充电完成条件
    if (vbus >= PRECHARGE_VOLTAGE_THRESHOLD) {
        // 母线电压达到阈值，切换到主继电器
        HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_SET);
        HAL_Delay(50);  // 等待主继电器可靠闭合

        // 断开预充电继电器
        HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);

        // 进入PFC启动阶段
        TransitionTo(CHARGE_STAGE_PFC_START);
        precharge_started = false;
        return;
    }

    // 超时检查
    if (GetElapsedTime(g_charge_status.stage_entry_time) >
        g_charge_config.timeout_precharge) {
        g_charge_status.fault_flags |= CHARGE_FAULT_PRECHARGE_TO;
        TransitionTo(CHARGE_STAGE_ERROR);
        precharge_started = false;
    }
}

/**
 * @brief PFC启动阶段处理
 * @details 启动PFC模块，将母线升压到800V
 *          检查：电压范围780-820V，输出稳定
 *          超时：5秒
 */
static void Stage_PFCStart_Handler(void)
{
    static bool pfc_started = false;

    // 首次进入：启动PFC
    if (!pfc_started) {
        if (!PFC_Start()) {
            g_charge_status.fault_flags |= CHARGE_FAULT_PFC_START_TO;
            TransitionTo(CHARGE_STAGE_ERROR);
            return;
        }
        pfc_started = true;
        return;
    }

    // 检查PFC软启动完成
    if (PFC_IsSoftStartComplete() && PFC_IsOutputStable()) {
        // 读取母线电压
        float vbus = PFC_GetBusVoltage();
        g_charge_status.bus_voltage = vbus;

        // 检查电压是否在目标范围内（780-820V）
        if (vbus >= PFC_VOLTAGE_MIN && vbus <= PFC_VOLTAGE_MAX) {
            // PFC启动成功，进入LLC启动阶段
            TransitionTo(CHARGE_STAGE_LLC_START);
            pfc_started = false;
            return;
        } else if (vbus > PFC_VOLTAGE_MAX) {
            // 母线过压
            g_charge_status.fault_flags |= CHARGE_FAULT_BUS_OV;
            TransitionTo(CHARGE_STAGE_ERROR);
            pfc_started = false;
            return;
        }
    }

    // 超时检查
    if (GetElapsedTime(g_charge_status.stage_entry_time) >
        g_charge_config.timeout_pfc_start) {
        g_charge_status.fault_flags |= CHARGE_FAULT_PFC_START_TO;
        TransitionTo(CHARGE_STAGE_ERROR);
        pfc_started = false;
    }
}

/**
 * @brief LLC启动阶段处理
 * @details 启动LLC模块（降压模式），800V → 125V
 *          检查：输入均压（C1≈C2≈400V），输出电压100-140V，输出均流
 *          超时：3秒
 */
static void Stage_LLCStart_Handler(void)
{
    static bool llc_started = false;

    // 首次进入：检查均压并启动LLC
    if (!llc_started) {
        // 检查输入均压（C1≈C2≈400V）
        float vc1 = LLC_GetInputVoltage1();
        float vc2 = LLC_GetInputVoltage2();
        float imbalance_v = fabs(vc1 - vc2);

        g_charge_status.llc_vc1 = vc1;
        g_charge_status.llc_vc2 = vc2;

        // 检查不平衡是否超过阈值
        if (imbalance_v > LLC_INPUT_IMBALANCE_MAX) {
            g_charge_status.fault_flags |= CHARGE_FAULT_IMBALANCE_V;
            TransitionTo(CHARGE_STAGE_ERROR);
            return;
        }

        // 设置LLC为恒流模式启动（软启动阶段）
        LLC_SetControlMode(LLC_MODE_CURRENT);
        LLC_SetCurrentRef(g_charge_config.charge_current_cc);

        // 启动LLC（正向/降压模式）
        if (!LLC_Start(LLC_DIR_FORWARD)) {
            g_charge_status.fault_flags |= CHARGE_FAULT_LLC_START_TO;
            TransitionTo(CHARGE_STAGE_ERROR);
            return;
        }

        llc_started = true;
        return;
    }

    // 检查LLC软启动完成
    if (LLC_IsSoftStartComplete()) {
        // 读取输出电压
        float vout = LLC_GetOutputVoltage();
        g_charge_status.battery_voltage = vout;

        // 检查输出电压范围
        if (vout >= LLC_OUTPUT_VOLTAGE_MIN && vout <= LLC_OUTPUT_VOLTAGE_MAX) {
            // 检查输入均压和输出均流
            bool input_balanced = LLC_IsInputBalanced();
            bool output_balanced = LLC_IsOutputBalanced();

            if (input_balanced && output_balanced) {
                // LLC启动成功，进入恒流充电阶段
                TransitionTo(CHARGE_STAGE_CC);
                llc_started = false;
                return;
            } else {
                // 均压均流失败
                g_charge_status.fault_flags |= CHARGE_FAULT_IMBALANCE_V;
                TransitionTo(CHARGE_STAGE_ERROR);
                llc_started = false;
                return;
            }
        }
    }

    // 超时检查
    if (GetElapsedTime(g_charge_status.stage_entry_time) >
        g_charge_config.timeout_llc_start) {
        g_charge_status.fault_flags |= CHARGE_FAULT_LLC_START_TO;
        TransitionTo(CHARGE_STAGE_ERROR);
        llc_started = false;
    }
}

/**
 * @brief 恒流充电阶段处理（CC - Constant Current）
 * @details 以恒定电流对电池充电，直到电压达到CV阈值
 *          控制模式：恒流
 *          目标电流：charge_current_cc（默认40A）
 *          转换条件：电池电压 >= charge_voltage_cv
 */
static void Stage_CC_Handler(void)
{
    // 设置LLC为恒流模式
    LLC_SetControlMode(LLC_MODE_CURRENT);

    // 功率限制检查
    if (g_charge_status.battery_power > g_charge_config.charge_power_max) {
        // 超过功率限制，降低电流以限制功率
        float i_limit = g_charge_config.charge_power_max /
                        g_charge_status.battery_voltage;

        // 确保电流不小于截止电流
        if (i_limit < g_charge_config.charge_current_cutoff) {
            i_limit = g_charge_config.charge_current_cutoff;
        }

        LLC_SetCurrentRef(i_limit);

        // 设置功率限制警告
        g_charge_status.warning_flags |= CHARGE_WARN_POWER_LIMIT;
    } else {
        // 正常恒流充电
        LLC_SetCurrentRef(g_charge_config.charge_current_cc);

        // 清除功率限制警告
        g_charge_status.warning_flags &= ~CHARGE_WARN_POWER_LIMIT;
    }

    // 检查是否达到CV转换条件
    if (g_charge_status.battery_voltage >= g_charge_config.charge_voltage_cv) {
        // 电压达到CV阈值，切换到恒压充电
        TransitionTo(CHARGE_STAGE_CV);
        return;
    }

    // 检查电池是否异常（电压过高但未触发CV）
    if (g_charge_status.battery_voltage > g_charge_config.protection_voltage_max) {
        g_charge_status.fault_flags |= CHARGE_FAULT_BATTERY_OV;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }
}

/**
 * @brief 恒压充电阶段处理（CV - Constant Voltage）
 * @details 以恒定电压对电池充电，电流逐渐下降
 *          控制模式：恒压
 *          目标电压：charge_voltage_cv（默认137.5V）
 *          转换条件：充电电流 <= charge_current_cutoff
 */
static void Stage_CV_Handler(void)
{
    // 设置LLC为恒压模式
    LLC_SetControlMode(LLC_MODE_VOLTAGE);
    LLC_SetVoltageRef(g_charge_config.charge_voltage_cv);

    // 检查电流是否降到截止值
    if (g_charge_status.battery_current <= g_charge_config.charge_current_cutoff) {
        // 电流降到截止值，充电基本完成，进入浮充阶段
        TransitionTo(CHARGE_STAGE_FLOAT);
        return;
    }

    // 功率限制检查（CV阶段一般不需要，但保险起见）
    if (g_charge_status.battery_power > g_charge_config.charge_power_max) {
        // 降低电压以限制功率
        float v_limit = g_charge_config.charge_power_max /
                        g_charge_status.battery_current;

        if (v_limit < g_charge_config.battery_voltage_nom) {
            v_limit = g_charge_config.battery_voltage_nom;
        }

        LLC_SetVoltageRef(v_limit);
        g_charge_status.warning_flags |= CHARGE_WARN_POWER_LIMIT;
    }
}

/**
 * @brief 浮充阶段处理（Float Charge）
 * @details 降低电压进行浮充维护，防止过充
 *          控制模式：恒压
 *          目标电压：charge_voltage_float（默认131.25V，即125V×1.05）
 *          持续时间：float_time_ms（默认1小时）
 */
static void Stage_Float_Handler(void)
{
    // 设置LLC为恒压模式，电压降低到浮充电压
    LLC_SetControlMode(LLC_MODE_VOLTAGE);
    LLC_SetVoltageRef(g_charge_config.charge_voltage_float);

    // 检查浮充时间
    if (GetElapsedTime(g_charge_status.stage_entry_time) >=
        g_charge_config.float_time_ms) {
        // 浮充完成，充电结束
        TransitionTo(CHARGE_STAGE_COMPLETE);
        return;
    }

    // 浮充阶段可以选择性地监测电流
    // 如果电流持续很低（如<0.5A），可以提前结束浮充
    if (g_charge_status.battery_current < 0.5f) {
        static uint32_t low_current_time = 0;

        if (low_current_time == 0) {
            low_current_time = HAL_GetTick();
        } else if (GetElapsedTime(low_current_time) > 600000) {  // 10分钟
            // 电流持续低于0.5A超过10分钟，提前完成
            TransitionTo(CHARGE_STAGE_COMPLETE);
            low_current_time = 0;
            return;
        }
    }
}

/******************************************************************************
 * 保护和检查函数实现
 ******************************************************************************/

/**
 * @brief 保护检查函数
 * @details 每个1kHz周期调用，检查各种保护条件
 */
static void Protection_Check(void)
{
    // 只在运行阶段进行保护检查
    if (g_charge_status.stage < CHARGE_STAGE_LLC_START) {
        return;
    }

    /* 1. 电池过压保护 */
    if (g_charge_status.battery_voltage > g_charge_config.protection_voltage_max) {
        g_charge_status.fault_flags |= CHARGE_FAULT_BATTERY_OV;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 2. 充电过流保护 */
    if (g_charge_status.battery_current > g_charge_config.protection_current_max) {
        g_charge_status.fault_flags |= CHARGE_FAULT_BATTERY_OC;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 3. 母线电压保护 */
    if (g_charge_status.bus_voltage > 850.0f) {
        g_charge_status.fault_flags |= CHARGE_FAULT_BUS_OV;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    if (g_charge_status.bus_voltage < 750.0f) {
        // 母线欠压（PFC可能故障）
        g_charge_status.fault_flags |= CHARGE_FAULT_BUS_UV;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 4. 电网状态检查 */
    if (!GridMonitor_IsGridOK()) {
        g_charge_status.fault_flags |= CHARGE_FAULT_GRID_FAIL;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 5. LLC输入均压检查 */
    float input_imbalance = LLC_GetInputImbalance();
    if (input_imbalance > g_charge_config.protection_imbalance_v) {
        g_charge_status.fault_flags |= CHARGE_FAULT_IMBALANCE_V;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 6. LLC输出均流检查 */
    float output_imbalance = LLC_GetOutputImbalance();
    if (output_imbalance > g_charge_config.protection_imbalance_i) {
        g_charge_status.fault_flags |= CHARGE_FAULT_IMBALANCE_I;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 7. PFC运行状态检查 */
    if (!PFC_IsRunning() || !PFC_IsOutputStable()) {
        g_charge_status.fault_flags |= CHARGE_FAULT_PFC_FAIL;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 8. LLC运行状态检查 */
    if (!LLC_IsRunning()) {
        g_charge_status.fault_flags |= CHARGE_FAULT_LLC_FAIL;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 9. 充电总超时检查 */
    uint32_t total_time = HAL_GetTick() - g_charge_status.charge_start_time;
    if (total_time > g_charge_config.timeout_charge_total) {
        g_charge_status.fault_flags |= CHARGE_FAULT_CHARGE_TIMEOUT;
        TransitionTo(CHARGE_STAGE_ERROR);
        return;
    }

    /* 10. 轻微不平衡警告（不触发故障） */
    if (input_imbalance > g_charge_config.protection_imbalance_v * 0.5f ||
        output_imbalance > g_charge_config.protection_imbalance_i * 0.5f) {
        g_charge_status.warning_flags |= CHARGE_WARN_IMBALANCE_MINOR;
    } else {
        g_charge_status.warning_flags &= ~CHARGE_WARN_IMBALANCE_MINOR;
    }
}

/**
 * @brief 更新测量数据
 */
static void Update_Measurements(void)
{
    // 电池侧测量（LLC输出）
    g_charge_status.battery_voltage = LLC_GetOutputVoltage();
    g_charge_status.battery_current = LLC_GetOutputCurrent();
    g_charge_status.battery_power = g_charge_status.battery_voltage *
                                     g_charge_status.battery_current;

    // 母线电压
    g_charge_status.bus_voltage = PFC_GetBusVoltage();

    // 电网参数
    g_charge_status.grid_voltage = GridMonitor_GetGridVoltage();
    g_charge_status.power_factor = PFC_GetPowerFactor();
}

/**
 * @brief 更新LLC均压均流状态
 */
static void Update_LLC_Balance_Status(void)
{
    // LLC输入电压（C1和C2）
    g_charge_status.llc_vc1 = LLC_GetInputVoltage1();
    g_charge_status.llc_vc2 = LLC_GetInputVoltage2();

    // LLC输出电流（两路并联）
    g_charge_status.llc_iout1 = LLC_GetOutputCurrent1();
    g_charge_status.llc_iout2 = LLC_GetOutputCurrent2();
}

/******************************************************************************
 * 辅助函数实现
 ******************************************************************************/

/**
 * @brief 状态转换函数
 * @param new_stage 新的充电阶段
 */
static void TransitionTo(ChargeStage_t new_stage)
{
    if (g_charge_status.stage != new_stage) {
        g_charge_status.stage = new_stage;
        g_charge_status.stage_entry_time = HAL_GetTick();
    }
}

/**
 * @brief 计算经过时间
 * @param start_time 起始时间戳
 * @return 经过的毫秒数
 */
static uint32_t GetElapsedTime(uint32_t start_time)
{
    uint32_t current_time = HAL_GetTick();

    // 处理时间戳溢出
    if (current_time >= start_time) {
        return current_time - start_time;
    } else {
        return (0xFFFFFFFF - start_time) + current_time + 1;
    }
}

/**
 * @brief 设置默认配置参数
 */
static void SetDefaultConfig(void)
{
    // 电池参数
    g_charge_config.battery_voltage_nom = DEFAULT_BATTERY_VOLTAGE_NOM;
    g_charge_config.battery_voltage_max = DEFAULT_BATTERY_VOLTAGE_MAX;
    g_charge_config.battery_voltage_min = DEFAULT_BATTERY_VOLTAGE_MIN;
    g_charge_config.battery_capacity_ah = DEFAULT_BATTERY_CAPACITY_AH;

    // 充电参数
    g_charge_config.charge_current_cc = DEFAULT_CHARGE_CURRENT_CC;
    g_charge_config.charge_voltage_cv = DEFAULT_CHARGE_VOLTAGE_CV;
    g_charge_config.charge_current_cutoff = DEFAULT_CHARGE_CURRENT_CUTOFF;
    g_charge_config.charge_voltage_float = DEFAULT_CHARGE_VOLTAGE_FLOAT;
    g_charge_config.charge_power_max = DEFAULT_CHARGE_POWER_MAX;
    g_charge_config.float_time_ms = DEFAULT_FLOAT_TIME_MS;

    // 超时设置
    g_charge_config.timeout_precharge = DEFAULT_TIMEOUT_PRECHARGE;
    g_charge_config.timeout_pfc_start = DEFAULT_TIMEOUT_PFC_START;
    g_charge_config.timeout_llc_start = DEFAULT_TIMEOUT_LLC_START;
    g_charge_config.timeout_charge_total = DEFAULT_TIMEOUT_CHARGE_TOTAL;

    // 保护阈值
    g_charge_config.protection_voltage_max = DEFAULT_PROTECTION_VOLTAGE_MAX;
    g_charge_config.protection_current_max = DEFAULT_PROTECTION_CURRENT_MAX;
    g_charge_config.protection_imbalance_v = DEFAULT_PROTECTION_IMBALANCE_V;
    g_charge_config.protection_imbalance_i = DEFAULT_PROTECTION_IMBALANCE_I;
}

/**
 * @brief 重置状态变量
 */
static void ResetStatus(void)
{
    g_charge_status.stage = CHARGE_STAGE_IDLE;
    g_charge_status.is_running = false;
    g_charge_status.stage_entry_time = 0;
    g_charge_status.charge_start_time = 0;

    g_charge_status.battery_voltage = 0.0f;
    g_charge_status.battery_current = 0.0f;
    g_charge_status.battery_power = 0.0f;
    g_charge_status.bus_voltage = 0.0f;
    g_charge_status.grid_voltage = 0.0f;
    g_charge_status.power_factor = 0.0f;

    g_charge_status.llc_vc1 = 0.0f;
    g_charge_status.llc_vc2 = 0.0f;
    g_charge_status.llc_iout1 = 0.0f;
    g_charge_status.llc_iout2 = 0.0f;

    g_charge_status.charge_time_sec = 0;
    g_charge_status.charge_ah = 0.0f;
    g_charge_status.charge_wh = 0.0f;
    g_charge_status.soc_estimate = 0.0f;

    g_charge_status.fault_flags = CHARGE_FAULT_NONE;
    g_charge_status.warning_flags = CHARGE_WARN_NONE;
}

/******************************************************************************
 * End of file
 ******************************************************************************/
