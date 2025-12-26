/******************************************************************************
 * @file    discharging_control.c
 * @brief   放电模式管理模块实现
 * @details 实现LLC升压+三相并网逆变器控制，包括并网同步和保护功能
 * @author  Generated for TI DSP280049 Platform
 * @date    2025-12-26
 ******************************************************************************/

#include "discharging_control.h"
#include "module_llc_bidir.h"
#include "module_grid_inv.h"
#include "grid_monitor.h"
#include "antiislanding.h"
#include "hal.h"
#include <string.h>
#include <math.h>

/******************************************************************************
 * 全局静态变量
 ******************************************************************************/

/**
 * @brief 放电状态全局变量
 */
static DischargeStatus_t g_discharge_status = {.stage = DISCHARGE_STAGE_IDLE};

/**
 * @brief 放电配置全局变量
 */
static DischargeConfig_t g_discharge_config = {0};

/**
 * @brief 模块初始化标志
 */
static bool g_initialized = false;

/**
 * @brief 上一次100Hz任务的时间戳（用于积分计算）
 */
static uint32_t g_last_100hz_time = 0;

/**
 * @brief 功率爬坡当前值（用于爬坡阶段）
 */
static float g_current_power_ramp = 0.0f;

/******************************************************************************
 * 内部辅助函数声明
 ******************************************************************************/

/* 阶段处理函数 */
static void Stage_LLCStart_Handler(void);
static void Stage_DCBusStable_Handler(void);
static void Stage_GridSyncing_Handler(void);
static void Stage_GridConnecting_Handler(void);
static void Stage_PowerRamping_Handler(void);
static void Stage_Running_Handler(void);

/* 保护和检查函数 */
static void Protection_Check(void);
static void Update_Measurements(void);
static void Update_LLC_Balance_Status(void);
static bool Check_Sync_Conditions(void);

/* 状态转换和辅助函数 */
static void TransitionTo(DischargeStage_t new_stage);
static uint32_t GetElapsedTime(uint32_t start_time);
static void SetDefaultConfig(void);
static void ResetStatus(void);

/******************************************************************************
 * 公共接口函数实现
 ******************************************************************************/

/**
 * @brief 放电模块初始化
 */
void Discharging_Init(void)
{
    // 初始化状态结构体
    memset(&g_discharge_status, 0, sizeof(DischargeStatus_t));
    g_discharge_status.stage = DISCHARGE_STAGE_IDLE;
    g_discharge_status.is_running = false;
    g_discharge_status.grid_connected = false;

    // 设置默认配置
    SetDefaultConfig();

    // 初始化防孤岛保护
    AntiIslanding_Init();

    // 初始化时间戳
    g_last_100hz_time = HAL_GetTick();

    // 初始化功率爬坡变量
    g_current_power_ramp = 0.0f;

    g_initialized = true;
}

/**
 * @brief 启动放电
 */
bool Discharging_Start(const DischargeConfig_t* config)
{
    // 检查初始化状态
    if (!g_initialized) {
        return false;
    }

    // 检查是否已经在运行
    if (g_discharge_status.is_running) {
        return false;
    }

    // 检查电网状态
    if (!GridMonitor_IsGridOK()) {
        g_discharge_status.fault_flags = DISCHARGE_FAULT_GRID_VOLTAGE;
        g_discharge_status.stage = DISCHARGE_STAGE_ERROR;
        return false;
    }

    // 应用配置（如果提供）
    if (config != NULL) {
        memcpy(&g_discharge_config, config, sizeof(DischargeConfig_t));
    } else {
        SetDefaultConfig();
    }

    // 重置状态
    ResetStatus();

    // 设置初始状态
    g_discharge_status.stage = DISCHARGE_STAGE_LLC_START;
    g_discharge_status.is_running = true;
    g_discharge_status.grid_connected = false;
    g_discharge_status.stage_entry_time = HAL_GetTick();
    g_discharge_status.discharge_start_time = HAL_GetTick();
    g_last_100hz_time = HAL_GetTick();
    g_current_power_ramp = 0.0f;

    return true;
}

/**
 * @brief 正常停止放电
 */
void Discharging_Stop(void)
{
    if (!g_discharge_status.is_running) {
        return;
    }

    // 如果已并网，需要安全脱网
    if (g_discharge_status.grid_connected) {
        /*--------------------------------------------------------------------
         * 安全脱网序列（符合并网标准）
         *--------------------------------------------------------------------*/
        // 步骤1: 功率降到0（使用快速下降速率）
        float power_step = g_discharge_config.power_ramp_down_rate / 10.0f;  // 100ms步进
        float current_power = g_discharge_status.active_power;

        while (current_power > 0.0f) {
            current_power -= power_step;
            if (current_power < 0.0f) {
                current_power = 0.0f;
            }
            GridInv_SetActivePower(current_power);
            HAL_Delay(100);  // 每100ms降一次
        }

        HAL_Delay(100);  // 确保功率降到0

        // 步骤2: 断开并网接触器
        HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_RESET);
        g_discharge_status.grid_connected = false;
        HAL_Delay(100);

        // 步骤3: 停止逆变器
        GridInv_Stop();
        HAL_Delay(100);

        // 步骤4: 禁用防孤岛保护
        AntiIslanding_Disable();
    } else {
        // 未并网，直接停止逆变器（如果运行中）
        if (GridInv_IsRunning()) {
            GridInv_Stop();
            HAL_Delay(100);
        }
    }

    // 步骤5: 停止LLC
    if (LLC_IsRunning()) {
        LLC_Stop();
        HAL_Delay(100);
    }

    // 重置状态
    g_discharge_status.is_running = false;
    g_discharge_status.grid_connected = false;
    g_discharge_status.stage = DISCHARGE_STAGE_IDLE;
    g_current_power_ramp = 0.0f;
}

/**
 * @brief 紧急停止放电（立即脱网）
 */
void Discharging_Abort(void)
{
    /*------------------------------------------------------------------------
     * 紧急脱网序列（符合电网故障快速脱网要求：<100ms）
     *------------------------------------------------------------------------*/
    // 立即断开并网接触器（最高优先级）
    HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_RESET);
    g_discharge_status.grid_connected = false;

    // 立即停止逆变器（不等待功率下降）
    GridInv_Stop();

    // 禁用防孤岛保护
    AntiIslanding_Disable();

    // 延时后停止LLC（避免母线过压）
    HAL_Delay(50);
    LLC_Stop();

    // 重置状态
    g_discharge_status.is_running = false;
    g_discharge_status.grid_connected = false;
    g_discharge_status.stage = DISCHARGE_STAGE_ERROR;
    g_current_power_ramp = 0.0f;
}

/**
 * @brief 放电控制任务 - 1kHz调用
 */
void Discharging_Run_1kHz(void)
{
    // 检查运行状态
    if (!g_discharge_status.is_running) {
        return;
    }

    // 更新测量数据
    Update_Measurements();

    // 更新LLC均压均流状态
    Update_LLC_Balance_Status();

    // 如果发生故障，紧急停止
    if (g_discharge_status.stage == DISCHARGE_STAGE_ERROR) {
        Discharging_Abort();
        return;
    }

    // 阶段状态机
    switch (g_discharge_status.stage) {
        case DISCHARGE_STAGE_LLC_START:
            Stage_LLCStart_Handler();
            break;

        case DISCHARGE_STAGE_DC_BUS_STABLE:
            Stage_DCBusStable_Handler();
            break;

        case DISCHARGE_STAGE_GRID_SYNCING:
            Stage_GridSyncing_Handler();
            break;

        case DISCHARGE_STAGE_GRID_CONNECTING:
            Stage_GridConnecting_Handler();
            break;

        case DISCHARGE_STAGE_POWER_RAMPING:
            Stage_PowerRamping_Handler();
            break;

        case DISCHARGE_STAGE_RUNNING:
            Stage_Running_Handler();
            break;

        case DISCHARGE_STAGE_COMPLETE:
            // 放电完成，保持当前状态
            break;

        case DISCHARGE_STAGE_ERROR:
            // 错误状态，等待外部处理
            Discharging_Abort();
            break;

        default:
            // 未知状态，强制停止
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_LLC_FAIL;
            TransitionTo(DISCHARGE_STAGE_ERROR);
            break;
    }

    // 保护检查（仅在并网后）
    if (g_discharge_status.grid_connected) {
        Protection_Check();
    }
}

/**
 * @brief 放电监测任务 - 100Hz调用
 */
void Discharging_Run_100Hz(void)
{
    if (!g_discharge_status.is_running) {
        return;
    }

    // 计算时间间隔（秒）
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - g_last_100hz_time) / 1000.0f;  // 转换为秒
    g_last_100hz_time = current_time;

    // 更新放电时间
    g_discharge_status.discharge_time_sec =
        (HAL_GetTick() - g_discharge_status.discharge_start_time) / 1000;

    // 如果已并网，更新并网时间
    if (g_discharge_status.grid_connected) {
        g_discharge_status.total_grid_time_sec =
            (HAL_GetTick() - g_discharge_status.grid_connect_time) / 1000;
    }

    // 统计仅在运行阶段（实际放电时）
    if (g_discharge_status.stage == DISCHARGE_STAGE_RUNNING ||
        g_discharge_status.stage == DISCHARGE_STAGE_POWER_RAMPING) {

        // 累积电池放电电量（Ah）- 使用梯形积分
        // Q = ∫I·dt
        g_discharge_status.discharge_ah +=
            g_discharge_status.battery_current * dt / 3600.0f;

        // 累积电池放电能量（Wh）- 从电池侧计算
        // E = ∫P·dt = ∫V·I·dt
        g_discharge_status.discharge_wh +=
            g_discharge_status.battery_power * dt / 3600.0f;

        // 累积输出到电网的能量（Wh）
        g_discharge_status.total_energy_export +=
            g_discharge_status.active_power * dt / 3600.0f;

        // 计算放电效率（输出能量/电池能量）
        if (g_discharge_status.discharge_wh > 0.01f) {
            g_discharge_status.efficiency =
                (g_discharge_status.total_energy_export /
                 g_discharge_status.discharge_wh) * 100.0f;
        }

        // 估算SOC（基于放出电量）
        if (g_discharge_config.battery_capacity_ah > 0) {
            float soc_decrement = (g_discharge_status.discharge_ah /
                                   g_discharge_config.battery_capacity_ah) * 100.0f;
            // 假设初始SOC为100%
            g_discharge_status.soc_estimate = 100.0f - soc_decrement;

            // 限制在0-100%
            if (g_discharge_status.soc_estimate < 0.0f) {
                g_discharge_status.soc_estimate = 0.0f;
            }
        }
    }

    /*------------------------------------------------------------------------
     * 电网监测和防孤岛检测（仅在并网后）
     *------------------------------------------------------------------------*/
    if (g_discharge_status.grid_connected) {
        // 防孤岛检测更新
        AntiIslanding_Update_100Hz();

        // 检查是否检测到孤岛（符合GB/T 19939标准）
        if (AntiIslanding_IsIslandDetected()) {
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_ISLANDING;
            g_discharge_status.islanding_detect_count++;

            // 检测到孤岛，立即脱网（<100ms）
            Discharging_Abort();
            return;
        }

        // 电网故障检查
        if (!GridMonitor_IsGridOK()) {
            GridFault_t fault = GridMonitor_GetFaultType();
            g_discharge_status.grid_fault_count++;

            // 设置对应故障标志
            if (fault == GRID_FAULT_OVERVOLTAGE ||
                fault == GRID_FAULT_UNDERVOLTAGE) {
                g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_VOLTAGE;
            } else if (fault == GRID_FAULT_OVERFREQUENCY ||
                       fault == GRID_FAULT_UNDERFREQUENCY) {
                g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_FREQUENCY;
            }

            // 电网故障，立即脱网
            Discharging_Abort();
            return;
        }

        // 电网质量警告（电压或频率接近边界）
        float v_diff = fabs(g_discharge_status.grid_voltage - GRID_VOLTAGE_NOMINAL);
        float f_diff = fabs(g_discharge_status.grid_frequency - GRID_FREQUENCY_NOMINAL);

        if (v_diff > 15.0f || f_diff > 0.3f) {
            g_discharge_status.warning_flags |= DISCHARGE_WARN_GRID_QUALITY;
        } else {
            g_discharge_status.warning_flags &= ~DISCHARGE_WARN_GRID_QUALITY;
        }
    }
}

/**
 * @brief 检查放电是否运行中
 */
bool Discharging_IsRunning(void)
{
    return g_discharge_status.is_running;
}

/**
 * @brief 检查是否已并网
 */
bool Discharging_IsGridConnected(void)
{
    return g_discharge_status.grid_connected;
}

/**
 * @brief 获取当前放电阶段
 */
DischargeStage_t Discharging_GetStage(void)
{
    return g_discharge_status.stage;
}

/**
 * @brief 获取放电状态指针
 */
const DischargeStatus_t* Discharging_GetStatus(void)
{
    return &g_discharge_status;
}

/**
 * @brief 获取放电进度
 */
float Discharging_GetProgress(void)
{
    // 基于SOC估算返回进度（100% → 0%，即放电进度）
    if (g_discharge_status.soc_estimate > 0) {
        return 100.0f - g_discharge_status.soc_estimate;  // 转换为放电进度
    }

    // 如果没有SOC，基于阶段返回粗略进度
    switch (g_discharge_status.stage) {
        case DISCHARGE_STAGE_IDLE:
        case DISCHARGE_STAGE_LLC_START:
        case DISCHARGE_STAGE_DC_BUS_STABLE:
        case DISCHARGE_STAGE_GRID_SYNCING:
        case DISCHARGE_STAGE_GRID_CONNECTING:
        case DISCHARGE_STAGE_POWER_RAMPING:
            return 0.0f;

        case DISCHARGE_STAGE_RUNNING:
            return 50.0f;  // 运行阶段假设50%

        case DISCHARGE_STAGE_COMPLETE:
            return 100.0f;

        default:
            return 0.0f;
    }
}

/**
 * @brief 设置目标输出功率
 */
void Discharging_SetPowerTarget(float p_target)
{
    // 限幅
    if (p_target > g_discharge_config.discharge_power_max) {
        p_target = g_discharge_config.discharge_power_max;
        g_discharge_status.warning_flags |= DISCHARGE_WARN_POWER_LIMIT;
    } else if (p_target < 0.0f) {
        p_target = 0.0f;
    } else {
        g_discharge_status.warning_flags &= ~DISCHARGE_WARN_POWER_LIMIT;
    }

    g_discharge_config.discharge_power_target = p_target;

    // 如果正在运行，立即更新
    if (g_discharge_status.stage == DISCHARGE_STAGE_RUNNING &&
        g_discharge_status.grid_connected) {
        GridInv_SetActivePower(p_target);
    }
}

/**
 * @brief 设置无功功率参考
 */
void Discharging_SetReactivePower(float q_target)
{
    g_discharge_config.reactive_power_ref = q_target;

    if (g_discharge_status.stage == DISCHARGE_STAGE_RUNNING &&
        g_discharge_status.grid_connected) {
        GridInv_SetReactivePower(q_target);
    }
}

/**
 * @brief 设置功率限制
 */
void Discharging_SetPowerLimit(float p_max)
{
    if (p_max > 0) {
        g_discharge_config.discharge_power_max = p_max;

        // 如果目标功率超过新限制，调整目标功率
        if (g_discharge_config.discharge_power_target > p_max) {
            Discharging_SetPowerTarget(p_max);
        }
    }
}

/**
 * @brief 设置功率爬坡速率
 */
void Discharging_SetRampRate(float rate)
{
    if (rate > 0) {
        g_discharge_config.power_ramp_rate = rate;
    }
}

/**
 * @brief 清除故障标志
 */
bool Discharging_ClearFaults(void)
{
    // 只有在停止状态下才能清除故障
    if (g_discharge_status.is_running) {
        return false;
    }

    g_discharge_status.fault_flags = DISCHARGE_FAULT_NONE;
    g_discharge_status.warning_flags = DISCHARGE_WARN_NONE;
    g_discharge_status.stage = DISCHARGE_STAGE_IDLE;

    return true;
}

/**
 * @brief 获取放电阶段名称字符串
 */
const char* Discharging_GetStageName(DischargeStage_t stage)
{
    switch (stage) {
        case DISCHARGE_STAGE_IDLE:            return "IDLE";
        case DISCHARGE_STAGE_LLC_START:       return "LLC_START";
        case DISCHARGE_STAGE_DC_BUS_STABLE:   return "DC_BUS_STABLE";
        case DISCHARGE_STAGE_GRID_SYNCING:    return "GRID_SYNCING";
        case DISCHARGE_STAGE_GRID_CONNECTING: return "GRID_CONNECTING";
        case DISCHARGE_STAGE_POWER_RAMPING:   return "POWER_RAMPING";
        case DISCHARGE_STAGE_RUNNING:         return "RUNNING";
        case DISCHARGE_STAGE_COMPLETE:        return "COMPLETE";
        case DISCHARGE_STAGE_ERROR:           return "ERROR";
        default:                              return "UNKNOWN";
    }
}

/**
 * @brief 获取故障描述字符串
 */
const char* Discharging_GetFaultString(uint32_t fault_code)
{
    if (fault_code == DISCHARGE_FAULT_NONE) return "No Fault";
    if (fault_code & DISCHARGE_FAULT_BATTERY_UV) return "Battery Undervoltage";
    if (fault_code & DISCHARGE_FAULT_BATTERY_OC) return "Battery Overcurrent";
    if (fault_code & DISCHARGE_FAULT_LLC_START_TO) return "LLC Start Timeout";
    if (fault_code & DISCHARGE_FAULT_LLC_IMBALANCE) return "LLC Imbalance";
    if (fault_code & DISCHARGE_FAULT_DCBUS_OV) return "DC Bus Overvoltage";
    if (fault_code & DISCHARGE_FAULT_DCBUS_UV) return "DC Bus Undervoltage";
    if (fault_code & DISCHARGE_FAULT_DCBUS_UNSTABLE) return "DC Bus Unstable";
    if (fault_code & DISCHARGE_FAULT_GRID_SYNC_TO) return "Grid Sync Timeout";
    if (fault_code & DISCHARGE_FAULT_GRID_SYNC_LOST) return "Grid Sync Lost";
    if (fault_code & DISCHARGE_FAULT_GRID_VOLTAGE) return "Grid Voltage Fault";
    if (fault_code & DISCHARGE_FAULT_GRID_FREQUENCY) return "Grid Frequency Fault";
    if (fault_code & DISCHARGE_FAULT_ISLANDING) return "Islanding Detected";
    if (fault_code & DISCHARGE_FAULT_OVERCURRENT) return "Output Overcurrent";
    if (fault_code & DISCHARGE_FAULT_DISCHARGE_TO) return "Discharge Timeout";
    if (fault_code & DISCHARGE_FAULT_INVERTER_FAIL) return "Inverter Fail";
    if (fault_code & DISCHARGE_FAULT_LLC_FAIL) return "LLC Fail";

    return "Multiple Faults";
}

/******************************************************************************
 * 阶段处理函数实现
 ******************************************************************************/

/**
 * @brief LLC启动阶段处理（升压模式）
 * @details 启动LLC模块升压，125V → 800V
 *          检查：输出电压750-850V，输入均流（Iin1≈Iin2），输出均压（Vout1≈Vout2≈400V）
 *          超时：3秒
 */
static void Stage_LLCStart_Handler(void)
{
    static bool llc_started = false;

    // 首次进入：设置LLC参数并启动
    if (!llc_started) {
        // 设置LLC为恒压模式，升压到800V
        LLC_SetControlMode(LLC_MODE_VOLTAGE);
        LLC_SetVoltageRef(g_discharge_config.dc_bus_voltage_ref);

        // 启动LLC（反向/升压方向）
        if (!LLC_Start(LLC_DIR_REVERSE)) {
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_LLC_START_TO;
            TransitionTo(DISCHARGE_STAGE_ERROR);
            return;
        }

        llc_started = true;
        return;
    }

    // 检查LLC软启动完成
    if (LLC_IsSoftStartComplete()) {
        // 读取输出电压（总800V）
        float vout = g_discharge_status.dc_bus_voltage;

        // 检查输出电压范围（750-850V）
        if (vout >= 750.0f && vout <= 850.0f) {
            // 检查输入均流和输出均压
            bool input_balanced = LLC_IsInputBalanced();   // Iin1≈Iin2
            bool output_balanced = LLC_IsOutputBalanced(); // Vout1≈Vout2≈400V

            if (input_balanced && output_balanced) {
                // LLC启动成功，进入母线稳定检查阶段
                TransitionTo(DISCHARGE_STAGE_DC_BUS_STABLE);
                llc_started = false;
                return;
            } else {
                // 均流均压失败
                g_discharge_status.fault_flags |= DISCHARGE_FAULT_LLC_IMBALANCE;
                TransitionTo(DISCHARGE_STAGE_ERROR);
                llc_started = false;
                return;
            }
        }
    }

    // 超时检查
    if (GetElapsedTime(g_discharge_status.stage_entry_time) >
        g_discharge_config.timeout_llc_start) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_LLC_START_TO;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        llc_started = false;
    }
}

/**
 * @brief 母线稳定检查阶段
 * @details 检查800V母线稳定性，要求电压在780-820V范围内持续500ms
 *          超时：3秒
 */
static void Stage_DCBusStable_Handler(void)
{
    static uint16_t stable_counter = 0;

    // 读取母线电压
    float vdc = g_discharge_status.dc_bus_voltage;

    // 检查母线电压是否在稳定范围内
    if (vdc >= DCBUS_VOLTAGE_STABLE_MIN && vdc <= DCBUS_VOLTAGE_STABLE_MAX) {
        stable_counter++;

        // 稳定超过配置时间（默认500ms = 500个1kHz周期）
        if (stable_counter >= DCBUS_STABLE_TIME_MS) {
            // 母线稳定，进入电网同步阶段
            TransitionTo(DISCHARGE_STAGE_GRID_SYNCING);
            stable_counter = 0;
            return;
        }
    } else {
        // 母线电压不在范围内，重置计数器
        stable_counter = 0;
    }

    // 超时检查
    if (GetElapsedTime(g_discharge_status.stage_entry_time) >
        g_discharge_config.timeout_dc_bus_stable) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_DCBUS_UNSTABLE;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        stable_counter = 0;
    }
}

/**
 * @brief 电网同步阶段处理
 * @details 启动逆变器，进行PLL锁相和同步检查
 *          同步条件：电压±10V，频率±0.2Hz，相位±5°
 *          同步保持时间：500ms
 *          超时：5秒
 */
static void Stage_GridSyncing_Handler(void)
{
    static bool inverter_started = false;
    static uint16_t sync_counter = 0;

    // 首次进入：启动逆变器（开始PLL锁相）
    if (!inverter_started) {
        // 检查电网状态
        if (!GridMonitor_IsGridOK()) {
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_VOLTAGE;
            TransitionTo(DISCHARGE_STAGE_ERROR);
            return;
        }

        // 启动逆变器（PLL开始同步）
        if (!GridInv_Start()) {
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_SYNC_TO;
            TransitionTo(DISCHARGE_STAGE_ERROR);
            return;
        }

        g_discharge_status.sync_start_time = HAL_GetTick();
        inverter_started = true;
        return;
    }

    // 检查PLL是否锁定
    if (GridInv_IsPLLLocked()) {
        // 检查同步条件（符合GB/T 19939标准）
        if (Check_Sync_Conditions()) {
            sync_counter++;

            // 同步条件保持足够时间（默认500ms）
            if (sync_counter >= g_discharge_config.sync_hold_time) {
                // 同步完成，进入并网投入阶段
                TransitionTo(DISCHARGE_STAGE_GRID_CONNECTING);
                inverter_started = false;
                sync_counter = 0;
                return;
            }
        } else {
            // 同步条件不满足，重置计数器
            sync_counter = 0;
        }
    } else {
        // PLL未锁定，重置计数器
        sync_counter = 0;
    }

    // 超时检查
    if (GetElapsedTime(g_discharge_status.stage_entry_time) >
        g_discharge_config.timeout_grid_sync) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_SYNC_TO;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        inverter_started = false;
        sync_counter = 0;
    }
}

/**
 * @brief 并网投入阶段处理
 * @details 闭合并网接触器，使能防孤岛保护
 *          检查并网后状态稳定
 *          超时：2秒
 */
static void Stage_GridConnecting_Handler(void)
{
    static bool contactor_closed = false;

    // 首次进入：闭合并网接触器
    if (!contactor_closed) {
        // 最后一次同步检查
        if (!Check_Sync_Conditions()) {
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_SYNC_LOST;
            TransitionTo(DISCHARGE_STAGE_ERROR);
            return;
        }

        // 闭合并网接触器
        HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_SET);
        HAL_Delay(100);  // 等待接触器可靠吸合

        // 标记并网状态
        g_discharge_status.grid_connected = true;
        g_discharge_status.grid_connect_time = HAL_GetTick();

        // 使能防孤岛保护（符合并网标准）
        AntiIslanding_Enable();

        contactor_closed = true;
        return;
    }

    // 等待一小段时间确认并网稳定（200ms）
    if (GetElapsedTime(g_discharge_status.stage_entry_time) >= 200) {
        // 检查并网后电网和同步状态
        if (GridMonitor_IsGridOK() && GridInv_IsPLLLocked()) {
            // 并网成功，进入功率爬坡阶段
            TransitionTo(DISCHARGE_STAGE_POWER_RAMPING);
            contactor_closed = false;
            return;
        } else {
            // 并网后检测到异常
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_SYNC_LOST;
            TransitionTo(DISCHARGE_STAGE_ERROR);
            contactor_closed = false;
            return;
        }
    }

    // 超时检查
    if (GetElapsedTime(g_discharge_status.stage_entry_time) >
        g_discharge_config.timeout_grid_connect) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_SYNC_LOST;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        contactor_closed = false;
    }
}

/**
 * @brief 功率爬坡阶段处理
 * @details 从0逐步增加到目标功率，避免对电网造成冲击
 *          爬坡速率：500W/s（可配置）
 *          超时：10秒
 */
static void Stage_PowerRamping_Handler(void)
{
    // 计算爬坡增量（1kHz调用，每次增加 ramp_rate/1000）
    float power_increment = g_discharge_config.power_ramp_rate / 1000.0f;  // W/ms

    // 增加当前功率
    g_current_power_ramp += power_increment;

    // 检查是否达到目标功率
    if (g_current_power_ramp >= g_discharge_config.discharge_power_target) {
        g_current_power_ramp = g_discharge_config.discharge_power_target;

        // 爬坡完成，进入正常运行阶段
        GridInv_SetActivePower(g_current_power_ramp);
        GridInv_SetReactivePower(g_discharge_config.reactive_power_ref);

        TransitionTo(DISCHARGE_STAGE_RUNNING);
        return;
    }

    // 设置逆变器功率参考（实时更新）
    GridInv_SetActivePower(g_current_power_ramp);
    GridInv_SetReactivePower(g_discharge_config.reactive_power_ref);

    // 超时检查（防止爬坡过程卡死）
    if (GetElapsedTime(g_discharge_status.stage_entry_time) >
        g_discharge_config.timeout_power_ramp) {
        // 超时但仍然进入运行状态（使用当前功率）
        TransitionTo(DISCHARGE_STAGE_RUNNING);
    }
}

/**
 * @brief 正常运行阶段处理
 * @details 维持目标功率输出，监测电池状态
 */
static void Stage_Running_Handler(void)
{
    // 维持目标功率输出
    GridInv_SetActivePower(g_discharge_config.discharge_power_target);
    GridInv_SetReactivePower(g_discharge_config.reactive_power_ref);

    // 检查电池电压（放电截止条件）
    if (g_discharge_status.battery_voltage < g_discharge_config.battery_voltage_min) {
        // 电池欠压，结束放电
        g_discharge_status.warning_flags |= DISCHARGE_WARN_BATTERY_LOW;
        TransitionTo(DISCHARGE_STAGE_COMPLETE);
        return;
    }

    // 可选：根据电池SOC动态调整功率
    if (g_discharge_status.soc_estimate > 0 && g_discharge_status.soc_estimate < 20.0f) {
        // SOC低于20%，设置低电量警告
        g_discharge_status.warning_flags |= DISCHARGE_WARN_BATTERY_LOW;
    } else {
        g_discharge_status.warning_flags &= ~DISCHARGE_WARN_BATTERY_LOW;
    }
}

/******************************************************************************
 * 保护和检查函数实现
 ******************************************************************************/

/**
 * @brief 保护检查函数（仅在并网后调用）
 * @details 每个1kHz周期调用，检查各种保护条件
 */
static void Protection_Check(void)
{
    static uint16_t sync_loss_counter = 0;

    /*------------------------------------------------------------------------
     * 1. 电池侧保护
     *------------------------------------------------------------------------*/
    // 电池欠压保护
    if (g_discharge_status.battery_voltage < g_discharge_config.battery_voltage_min) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_BATTERY_UV;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    // 电池过流保护
    if (g_discharge_status.battery_current > g_discharge_config.discharge_current_max) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_BATTERY_OC;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    /*------------------------------------------------------------------------
     * 2. 母线电压保护
     *------------------------------------------------------------------------*/
    float vdc = g_discharge_status.dc_bus_voltage;

    if (vdc > g_discharge_config.protection_dcbus_ov) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_DCBUS_OV;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    if (vdc < g_discharge_config.protection_dcbus_uv) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_DCBUS_UV;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    /*------------------------------------------------------------------------
     * 3. LLC均流均压保护
     *------------------------------------------------------------------------*/
    float input_imbalance = LLC_GetInputImbalance();    // |Iin1-Iin2|
    float output_imbalance = LLC_GetOutputImbalance();  // |Vout1-Vout2|

    if (input_imbalance > g_discharge_config.protection_imbalance_i ||
        output_imbalance > g_discharge_config.protection_imbalance_v) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_LLC_IMBALANCE;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    // 轻微不平衡警告
    if (input_imbalance > g_discharge_config.protection_imbalance_i * 0.5f ||
        output_imbalance > g_discharge_config.protection_imbalance_v * 0.5f) {
        g_discharge_status.warning_flags |= DISCHARGE_WARN_IMBALANCE_MINOR;
    } else {
        g_discharge_status.warning_flags &= ~DISCHARGE_WARN_IMBALANCE_MINOR;
    }

    /*------------------------------------------------------------------------
     * 4. 同步丢失检查（关键保护，<100ms脱网）
     *------------------------------------------------------------------------*/
    if (!GridInv_IsPLLLocked()) {
        sync_loss_counter++;

        // 连续丢失超过100ms（100个1kHz周期）
        if (sync_loss_counter >= 100) {
            g_discharge_status.fault_flags |= DISCHARGE_FAULT_GRID_SYNC_LOST;
            g_discharge_status.sync_loss_count++;
            TransitionTo(DISCHARGE_STAGE_ERROR);
            sync_loss_counter = 0;
            return;
        }
    } else {
        // PLL锁定，重置计数器
        sync_loss_counter = 0;
    }

    /*------------------------------------------------------------------------
     * 5. 输出过流保护
     *------------------------------------------------------------------------*/
    if (g_discharge_status.output_current > g_discharge_config.protection_current_max) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_OVERCURRENT;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    /*------------------------------------------------------------------------
     * 6. 逆变器运行状态检查
     *------------------------------------------------------------------------*/
    if (!GridInv_IsRunning()) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_INVERTER_FAIL;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    /*------------------------------------------------------------------------
     * 7. LLC运行状态检查
     *------------------------------------------------------------------------*/
    if (!LLC_IsRunning()) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_LLC_FAIL;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }

    /*------------------------------------------------------------------------
     * 8. 放电总超时检查
     *------------------------------------------------------------------------*/
    uint32_t total_time = HAL_GetTick() - g_discharge_status.discharge_start_time;
    if (total_time > g_discharge_config.timeout_discharge_total) {
        g_discharge_status.fault_flags |= DISCHARGE_FAULT_DISCHARGE_TO;
        TransitionTo(DISCHARGE_STAGE_ERROR);
        return;
    }
}

/**
 * @brief 更新测量数据
 */
static void Update_Measurements(void)
{
    // 电池侧测量（LLC输入）
    g_discharge_status.battery_voltage = LLC_GetInputVoltage();
    g_discharge_status.battery_current = LLC_GetInputCurrent();
    g_discharge_status.battery_power = g_discharge_status.battery_voltage *
                                        g_discharge_status.battery_current;

    // 母线电压（LLC输出，800V）
    g_discharge_status.dc_bus_voltage = LLC_GetOutputVoltage();

    // 电网参数（来自逆变器）
    g_discharge_status.grid_voltage = GridInv_GetGridVoltage();
    g_discharge_status.grid_frequency = GridInv_GetGridFrequency();
    g_discharge_status.phase_difference = GridInv_GetPhaseDifference();

    // 逆变器输出
    g_discharge_status.active_power = GridInv_GetActivePower();
    g_discharge_status.reactive_power = GridInv_GetReactivePower();
    g_discharge_status.output_current = GridInv_GetOutputCurrent();

    // 功率因数计算
    float apparent_power = sqrtf(g_discharge_status.active_power *
                                 g_discharge_status.active_power +
                                 g_discharge_status.reactive_power *
                                 g_discharge_status.reactive_power);
    if (apparent_power > 0.01f) {
        g_discharge_status.power_factor = g_discharge_status.active_power / apparent_power;
    }
}

/**
 * @brief 更新LLC均流均压状态
 */
static void Update_LLC_Balance_Status(void)
{
    // LLC输入电流（两路并联）
    g_discharge_status.llc_iin1 = LLC_GetInputCurrent1();
    g_discharge_status.llc_iin2 = LLC_GetInputCurrent2();

    // LLC输出电压（两路串联，各400V）
    g_discharge_status.llc_vout1 = LLC_GetOutputVoltage1();
    g_discharge_status.llc_vout2 = LLC_GetOutputVoltage2();
}

/**
 * @brief 检查电网同步条件
 * @details 符合GB/T 19939和IEEE 1547标准
 * @return true-满足同步条件, false-不满足
 */
static bool Check_Sync_Conditions(void)
{
    // 检查电压同步（电网电压 vs 标称电压）
    float v_diff = fabs(g_discharge_status.grid_voltage - GRID_VOLTAGE_NOMINAL);
    bool voltage_synced = (v_diff < g_discharge_config.sync_voltage_tolerance);

    // 检查频率同步（电网频率 vs 标称频率）
    float f_diff = fabs(g_discharge_status.grid_frequency - GRID_FREQUENCY_NOMINAL);
    bool freq_synced = (f_diff < g_discharge_config.sync_freq_tolerance);

    // 检查相位同步
    float phase_diff = fabs(g_discharge_status.phase_difference);
    bool phase_synced = (phase_diff < g_discharge_config.sync_phase_tolerance);

    // 所有条件都满足才算同步成功
    return (voltage_synced && freq_synced && phase_synced);
}

/******************************************************************************
 * 辅助函数实现
 ******************************************************************************/

/**
 * @brief 状态转换函数
 * @param new_stage 新的放电阶段
 */
static void TransitionTo(DischargeStage_t new_stage)
{
    if (g_discharge_status.stage != new_stage) {
        g_discharge_status.stage = new_stage;
        g_discharge_status.stage_entry_time = HAL_GetTick();
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
    g_discharge_config.battery_voltage_nom = DEFAULT_BATTERY_VOLTAGE_NOM_DIS;
    g_discharge_config.battery_voltage_min = DEFAULT_BATTERY_VOLTAGE_MIN_DIS;
    g_discharge_config.battery_voltage_max = DEFAULT_BATTERY_VOLTAGE_MAX_DIS;
    g_discharge_config.battery_capacity_ah = DEFAULT_BATTERY_CAPACITY_AH_DIS;

    // 放电参数
    g_discharge_config.discharge_power_target = DEFAULT_DISCHARGE_POWER_TARGET;
    g_discharge_config.discharge_power_max = DEFAULT_DISCHARGE_POWER_MAX;
    g_discharge_config.discharge_current_max = DEFAULT_DISCHARGE_CURRENT_MAX;

    // 并网参数
    g_discharge_config.dc_bus_voltage_ref = DEFAULT_DC_BUS_VOLTAGE_REF;
    g_discharge_config.reactive_power_ref = DEFAULT_REACTIVE_POWER_REF;
    g_discharge_config.power_ramp_rate = DEFAULT_POWER_RAMP_RATE;
    g_discharge_config.power_ramp_down_rate = DEFAULT_POWER_RAMP_DOWN_RATE;

    // 同步参数
    g_discharge_config.sync_voltage_tolerance = DEFAULT_SYNC_VOLTAGE_TOLERANCE;
    g_discharge_config.sync_freq_tolerance = DEFAULT_SYNC_FREQ_TOLERANCE;
    g_discharge_config.sync_phase_tolerance = DEFAULT_SYNC_PHASE_TOLERANCE;
    g_discharge_config.sync_hold_time = DEFAULT_SYNC_HOLD_TIME;

    // 超时设置
    g_discharge_config.timeout_llc_start = DEFAULT_TIMEOUT_LLC_START_DIS;
    g_discharge_config.timeout_dc_bus_stable = DEFAULT_TIMEOUT_DC_BUS_STABLE;
    g_discharge_config.timeout_grid_sync = DEFAULT_TIMEOUT_GRID_SYNC;
    g_discharge_config.timeout_grid_connect = DEFAULT_TIMEOUT_GRID_CONNECT;
    g_discharge_config.timeout_power_ramp = DEFAULT_TIMEOUT_POWER_RAMP;
    g_discharge_config.timeout_discharge_total = DEFAULT_TIMEOUT_DISCHARGE_TOTAL;

    // 保护阈值
    g_discharge_config.protection_current_max = DEFAULT_PROTECTION_CURRENT_MAX;
    g_discharge_config.protection_dcbus_ov = DEFAULT_PROTECTION_DCBUS_OV;
    g_discharge_config.protection_dcbus_uv = DEFAULT_PROTECTION_DCBUS_UV;
    g_discharge_config.protection_imbalance_i = DEFAULT_PROTECTION_IMBALANCE_I;
    g_discharge_config.protection_imbalance_v = DEFAULT_PROTECTION_IMBALANCE_V;
}

/**
 * @brief 重置状态变量
 */
static void ResetStatus(void)
{
    g_discharge_status.stage = DISCHARGE_STAGE_IDLE;
    g_discharge_status.is_running = false;
    g_discharge_status.grid_connected = false;

    g_discharge_status.stage_entry_time = 0;
    g_discharge_status.discharge_start_time = 0;
    g_discharge_status.sync_start_time = 0;
    g_discharge_status.grid_connect_time = 0;

    g_discharge_status.battery_voltage = 0.0f;
    g_discharge_status.battery_current = 0.0f;
    g_discharge_status.battery_power = 0.0f;
    g_discharge_status.dc_bus_voltage = 0.0f;

    g_discharge_status.llc_iin1 = 0.0f;
    g_discharge_status.llc_iin2 = 0.0f;
    g_discharge_status.llc_vout1 = 0.0f;
    g_discharge_status.llc_vout2 = 0.0f;

    g_discharge_status.grid_voltage = 0.0f;
    g_discharge_status.grid_frequency = 0.0f;
    g_discharge_status.phase_difference = 0.0f;

    g_discharge_status.active_power = 0.0f;
    g_discharge_status.reactive_power = 0.0f;
    g_discharge_status.output_current = 0.0f;
    g_discharge_status.power_factor = 0.0f;

    g_discharge_status.discharge_time_sec = 0;
    g_discharge_status.discharge_ah = 0.0f;
    g_discharge_status.discharge_wh = 0.0f;
    g_discharge_status.total_energy_export = 0.0f;
    g_discharge_status.efficiency = 0.0f;
    g_discharge_status.soc_estimate = 100.0f;  // 假设初始满电

    g_discharge_status.sync_loss_count = 0;
    g_discharge_status.grid_fault_count = 0;
    g_discharge_status.islanding_detect_count = 0;
    g_discharge_status.total_grid_time_sec = 0;

    g_discharge_status.fault_flags = DISCHARGE_FAULT_NONE;
    g_discharge_status.warning_flags = DISCHARGE_WARN_NONE;
}

/******************************************************************************
 * End of file
 ******************************************************************************/
