/******************************************************************************
 * @file    system_manager.c
 * @brief   顶层系统状态机管理实现
 * @details 实现PFC + 双交错LLC(IPOS) + 并网逆变器系统的完整状态机逻辑
 * @author  Generated for TI DSP280049 Platform
 * @date    2025-12-26
 ******************************************************************************/

#include "system_manager.h"
#include "module_pfc.h"          // PFC模块接口
#include "module_llc_bidir.h"    // 双向LLC模块接口
#include "module_grid_inv.h"     // 并网逆变器模块接口
#include "grid_monitor.h"        // 电网监测接口
#include "hal.h"                 // HAL层（GPIO, 时间戳等）

/******************************************************************************
 * 全局变量
 ******************************************************************************/

/**
 * @brief 系统状态机全局上下文
 */
static SystemContext_t g_sys_ctx;

/******************************************************************************
 * 内部辅助函数声明
 ******************************************************************************/

static void TransitionToState(SystemState_t new_state);
static void ResetSubStates(void);
static uint32_t GetElapsedTime(uint32_t start_time);
static void ShutdownAllModules(void);
static void State_Idle_Handler(void);
static void State_Charging_Handler(void);
static void State_Discharging_Handler(void);
static void State_Fault_Handler(void);
static void State_Stopping_Handler(void);

/******************************************************************************
 * 系统初始化
 ******************************************************************************/

/**
 * @brief 系统初始化
 */
void System_Init(void)
{
    // 初始化上下文结构体
    g_sys_ctx.state = SYS_STATE_IDLE;
    g_sys_ctx.mode = SYS_MODE_NONE;
    g_sys_ctx.charge_sub = CHARGE_IDLE;
    g_sys_ctx.discharge_sub = DISCHARGE_IDLE;
    g_sys_ctx.state_entry_time = HAL_GetTick();
    g_sys_ctx.substep_entry_time = HAL_GetTick();
    g_sys_ctx.fault_code = FAULT_NONE;
    g_sys_ctx.emergency_stop = false;
    g_sys_ctx.stop_requested = false;
    g_sys_ctx.charge_count = 0;
    g_sys_ctx.discharge_count = 0;
    g_sys_ctx.fault_count = 0;

    // 初始化所有功能模块
    if (!PFC_Init()) {
        System_TriggerFault(FAULT_PFC_INIT_FAIL);
        return;
    }

    if (!LLC_Init()) {
        System_TriggerFault(FAULT_LLC_INIT_FAIL);
        return;
    }

    if (!GridInv_Init()) {
        System_TriggerFault(FAULT_GRID_INV_INIT_FAIL);
        return;
    }

    // 初始化GPIO（继电器、接触器等）
    HAL_GPIO_Init(GPIO_RELAY_PRECHARGE, GPIO_MODE_OUTPUT);
    HAL_GPIO_Init(GPIO_RELAY_MAIN, GPIO_MODE_OUTPUT);
    HAL_GPIO_Init(GPIO_GRID_CONTACTOR, GPIO_MODE_OUTPUT);

    // 确保所有继电器初始状态为断开
    HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_RESET);
}

/******************************************************************************
 * 主状态机（1kHz周期调用）
 ******************************************************************************/

/**
 * @brief 主状态机周期调用函数
 */
void System_StateMachine_1kHz(void)
{
    // 紧急停机处理（优先级最高）
    if (g_sys_ctx.emergency_stop && g_sys_ctx.state != SYS_STATE_FAULT) {
        System_TriggerFault(FAULT_EMERGENCY_STOP);
        return;
    }

    // 顶层状态机分发
    switch (g_sys_ctx.state) {
        case SYS_STATE_IDLE:
            State_Idle_Handler();
            break;

        case SYS_STATE_CHARGING:
            State_Charging_Handler();
            break;

        case SYS_STATE_DISCHARGING:
            State_Discharging_Handler();
            break;

        case SYS_STATE_FAULT:
            State_Fault_Handler();
            break;

        case SYS_STATE_STOPPING:
            State_Stopping_Handler();
            break;

        default:
            // 未知状态，强制回到IDLE
            TransitionToState(SYS_STATE_IDLE);
            break;
    }
}

/******************************************************************************
 * 顶层状态处理函数
 ******************************************************************************/

/**
 * @brief IDLE状态处理
 */
static void State_Idle_Handler(void)
{
    // 在IDLE状态下，等待外部命令
    // 确保所有模块已停止
    if (PFC_IsRunning() || LLC_IsRunning() || GridInv_IsRunning()) {
        ShutdownAllModules();
    }

    // 确保所有继电器断开
    HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_RESET);
}

/**
 * @brief CHARGING状态处理
 */
static void State_Charging_Handler(void)
{
    // 检查停机请求
    if (g_sys_ctx.stop_requested) {
        g_sys_ctx.charge_sub = CHARGE_STOPPING;
        TransitionToState(SYS_STATE_STOPPING);
        return;
    }

    // 充电子状态机
    switch (g_sys_ctx.charge_sub) {

        /*--------------------------------------------------------------------
         * 预充电阶段
         *--------------------------------------------------------------------*/
        case CHARGE_PRECHARGE:
        {
            // 首次进入：检查电网，启动预充电
            if (GetElapsedTime(g_sys_ctx.substep_entry_time) == 0) {
                if (!GridMonitor_IsGridOK()) {
                    System_TriggerFault(FAULT_GRID_ABNORMAL);
                    return;
                }
                // 接通预充电继电器
                HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_SET);
                g_sys_ctx.substep_entry_time = HAL_GetTick();
            }

            // 检查预充电电压
            float bus_voltage = PFC_GetBusVoltage();  // 读取母线电压
            if (bus_voltage >= VOLTAGE_PRECHARGE_THRESHOLD) {
                // 预充电完成，切换到主继电器
                HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_SET);
                HAL_Delay(DELAY_MAIN_RELAY_MS);  // 等待主继电器闭合
                HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);

                // 进入PFC启动阶段
                g_sys_ctx.charge_sub = CHARGE_STARTING_PFC;
                g_sys_ctx.substep_entry_time = HAL_GetTick();
            }
            // 超时检查
            else if (GetElapsedTime(g_sys_ctx.substep_entry_time) > TIMEOUT_PRECHARGE_MS) {
                System_TriggerFault(FAULT_PRECHARGE_TIMEOUT);
                return;
            }
            break;
        }

        /*--------------------------------------------------------------------
         * PFC启动阶段
         *--------------------------------------------------------------------*/
        case CHARGE_STARTING_PFC:
        {
            // 首次进入：启动PFC
            if (GetElapsedTime(g_sys_ctx.substep_entry_time) == 0) {
                if (!PFC_Start()) {
                    System_TriggerFault(FAULT_PFC_START_TIMEOUT);
                    return;
                }
                g_sys_ctx.substep_entry_time = HAL_GetTick();
            }

            // 检查PFC输出是否稳定（800V）
            if (PFC_IsOutputStable()) {
                float bus_voltage = PFC_GetBusVoltage();
                // 验证母线电压在合理范围（800V ± 10%）
                if (bus_voltage >= 720.0f && bus_voltage <= 880.0f) {
                    // PFC启动成功，检查C1/C2均压
                    if (LLC_IsBalanced()) {
                        // 进入LLC启动阶段
                        g_sys_ctx.charge_sub = CHARGE_STARTING_LLC;
                        g_sys_ctx.substep_entry_time = HAL_GetTick();
                    } else {
                        System_TriggerFault(FAULT_LLC_IMBALANCE);
                        return;
                    }
                } else {
                    System_TriggerFault(FAULT_PFC_OUTPUT_UNSTABLE);
                    return;
                }
            }
            // 超时检查
            else if (GetElapsedTime(g_sys_ctx.substep_entry_time) > TIMEOUT_PFC_START_MS) {
                System_TriggerFault(FAULT_PFC_START_TIMEOUT);
                return;
            }
            break;
        }

        /*--------------------------------------------------------------------
         * LLC启动阶段（降压模式）
         *--------------------------------------------------------------------*/
        case CHARGE_STARTING_LLC:
        {
            // 首次进入：启动LLC（正向/降压模式）
            if (GetElapsedTime(g_sys_ctx.substep_entry_time) == 0) {
                if (!LLC_Start(LLC_DIR_FORWARD)) {
                    System_TriggerFault(FAULT_LLC_START_TIMEOUT);
                    return;
                }
                g_sys_ctx.substep_entry_time = HAL_GetTick();
            }

            // 检查LLC软启动是否完成
            if (LLC_IsSoftStartComplete()) {
                float llc_output = LLC_GetOutputVoltage();
                // 验证输出电压在合理范围（125V ± 5%）
                if (llc_output >= 118.75f && llc_output <= 131.25f) {
                    // 检查均流均压
                    if (LLC_IsBalanced()) {
                        // 充电模式启动完成
                        g_sys_ctx.charge_sub = CHARGE_RUNNING;
                        g_sys_ctx.substep_entry_time = HAL_GetTick();
                        g_sys_ctx.charge_count++;  // 统计充电次数
                    } else {
                        System_TriggerFault(FAULT_LLC_IMBALANCE);
                        return;
                    }
                } else {
                    System_TriggerFault(FAULT_LLC_START_TIMEOUT);
                    return;
                }
            }
            // 超时检查
            else if (GetElapsedTime(g_sys_ctx.substep_entry_time) > TIMEOUT_LLC_START_MS) {
                System_TriggerFault(FAULT_LLC_START_TIMEOUT);
                return;
            }
            break;
        }

        /*--------------------------------------------------------------------
         * 充电运行状态
         *--------------------------------------------------------------------*/
        case CHARGE_RUNNING:
        {
            // 持续监测运行状态
            // 1. 检查电网状态
            if (!GridMonitor_IsGridOK()) {
                System_TriggerFault(FAULT_GRID_LOST);
                return;
            }

            // 2. 检查PFC运行状态
            if (!PFC_IsRunning() || !PFC_IsOutputStable()) {
                System_TriggerFault(FAULT_PFC_OUTPUT_UNSTABLE);
                return;
            }

            // 3. 检查LLC运行状态
            if (!LLC_IsRunning()) {
                System_TriggerFault(FAULT_MODULE_RUNTIME_ERROR);
                return;
            }

            // 4. 检查均流均压
            if (!LLC_IsBalanced()) {
                System_TriggerFault(FAULT_LLC_IMBALANCE);
                return;
            }

            // 5. 检查母线电压
            float bus_voltage = PFC_GetBusVoltage();
            if (bus_voltage > 880.0f) {
                System_TriggerFault(FAULT_BUS_OVERVOLTAGE);
                return;
            } else if (bus_voltage < 720.0f) {
                System_TriggerFault(FAULT_BUS_UNDERVOLTAGE);
                return;
            }

            // 正常运行，无异常
            break;
        }

        /*--------------------------------------------------------------------
         * 充电停机（由STOPPING状态处理）
         *--------------------------------------------------------------------*/
        case CHARGE_STOPPING:
            // 此状态由State_Stopping_Handler处理
            break;

        default:
            // 未知子状态，触发故障
            System_TriggerFault(FAULT_MODULE_RUNTIME_ERROR);
            break;
    }
}

/**
 * @brief DISCHARGING状态处理
 */
static void State_Discharging_Handler(void)
{
    // 检查停机请求
    if (g_sys_ctx.stop_requested) {
        g_sys_ctx.discharge_sub = DISCHARGE_STOPPING;
        TransitionToState(SYS_STATE_STOPPING);
        return;
    }

    // 放电子状态机
    switch (g_sys_ctx.discharge_sub) {

        /*--------------------------------------------------------------------
         * LLC启动阶段（升压模式）
         *--------------------------------------------------------------------*/
        case DISCHARGE_STARTING_LLC:
        {
            // 首次进入：检查电池电压，启动LLC
            if (GetElapsedTime(g_sys_ctx.substep_entry_time) == 0) {
                // 检查电池电压
                float battery_voltage = LLC_GetOutputVoltage();  // 此时读取的是输入侧（电池侧）
                if (battery_voltage < VOLTAGE_BATTERY_MIN) {
                    System_TriggerFault(FAULT_BATTERY_VOLTAGE_LOW);
                    return;
                }

                // 启动LLC（反向/升压模式）
                if (!LLC_Start(LLC_DIR_REVERSE)) {
                    System_TriggerFault(FAULT_LLC_START_TIMEOUT);
                    return;
                }
                g_sys_ctx.substep_entry_time = HAL_GetTick();
            }

            // 检查LLC软启动是否完成
            if (LLC_IsSoftStartComplete()) {
                float llc_output = LLC_GetOutputVoltage();  // 此时读取的是输出侧（800V母线）
                // 验证输出电压在合理范围（800V ± 10%）
                if (llc_output >= 720.0f && llc_output <= 880.0f) {
                    // 检查均流均压
                    if (LLC_IsBalanced()) {
                        // LLC启动成功，进入电网同步阶段
                        g_sys_ctx.discharge_sub = DISCHARGE_GRID_SYNCING;
                        g_sys_ctx.substep_entry_time = HAL_GetTick();
                    } else {
                        System_TriggerFault(FAULT_LLC_IMBALANCE);
                        return;
                    }
                } else {
                    System_TriggerFault(FAULT_LLC_START_TIMEOUT);
                    return;
                }
            }
            // 超时检查
            else if (GetElapsedTime(g_sys_ctx.substep_entry_time) > TIMEOUT_LLC_START_MS) {
                System_TriggerFault(FAULT_LLC_START_TIMEOUT);
                return;
            }
            break;
        }

        /*--------------------------------------------------------------------
         * 电网同步阶段
         *--------------------------------------------------------------------*/
        case DISCHARGE_GRID_SYNCING:
        {
            // 首次进入：检查电网，启动逆变器
            if (GetElapsedTime(g_sys_ctx.substep_entry_time) == 0) {
                if (!GridMonitor_IsGridOK()) {
                    System_TriggerFault(FAULT_GRID_ABNORMAL);
                    return;
                }

                // 启动逆变器（PLL开始同步）
                if (!GridInv_Start()) {
                    System_TriggerFault(FAULT_GRID_SYNC_TIMEOUT);
                    return;
                }
                g_sys_ctx.substep_entry_time = HAL_GetTick();
            }

            // 检查PLL是否锁定
            if (GridInv_IsGridSynced()) {
                // PLL锁定后需要稳定一段时间
                if (GetElapsedTime(g_sys_ctx.substep_entry_time) >= DELAY_GRID_SYNC_STABLE_MS) {
                    // 进入并网投入阶段
                    g_sys_ctx.discharge_sub = DISCHARGE_GRID_CONNECTING;
                    g_sys_ctx.substep_entry_time = HAL_GetTick();
                }
            }
            // 超时检查
            else if (GetElapsedTime(g_sys_ctx.substep_entry_time) > TIMEOUT_GRID_SYNC_MS) {
                System_TriggerFault(FAULT_GRID_SYNC_TIMEOUT);
                return;
            }
            break;
        }

        /*--------------------------------------------------------------------
         * 并网投入阶段
         *--------------------------------------------------------------------*/
        case DISCHARGE_GRID_CONNECTING:
        {
            // 首次进入：闭合并网接触器
            if (GetElapsedTime(g_sys_ctx.substep_entry_time) == 0) {
                // 闭合并网接触器
                HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_SET);
                g_sys_ctx.substep_entry_time = HAL_GetTick();
            }

            // 功率爬坡逻辑（此处简化，实际应由逆变器模块内部实现）
            // 等待功率爬坡完成（假设3秒）
            if (GetElapsedTime(g_sys_ctx.substep_entry_time) >= 3000) {
                // 检查逆变器运行状态
                if (GridInv_IsRunning() && GridInv_IsGridSynced()) {
                    // 并网成功，进入运行状态
                    g_sys_ctx.discharge_sub = DISCHARGE_RUNNING;
                    g_sys_ctx.substep_entry_time = HAL_GetTick();
                    g_sys_ctx.discharge_count++;  // 统计放电次数
                } else {
                    System_TriggerFault(FAULT_MODULE_RUNTIME_ERROR);
                    return;
                }
            }
            // 超时检查
            else if (GetElapsedTime(g_sys_ctx.substep_entry_time) > TIMEOUT_GRID_CONNECT_MS) {
                System_TriggerFault(FAULT_GRID_SYNC_TIMEOUT);
                return;
            }
            break;
        }

        /*--------------------------------------------------------------------
         * 放电运行状态
         *--------------------------------------------------------------------*/
        case DISCHARGE_RUNNING:
        {
            // 持续监测运行状态
            // 1. 检查电网状态
            if (!GridMonitor_IsGridOK()) {
                System_TriggerFault(FAULT_GRID_LOST);
                return;
            }

            // 2. 检查逆变器运行状态
            if (!GridInv_IsRunning() || !GridInv_IsGridSynced()) {
                System_TriggerFault(FAULT_MODULE_RUNTIME_ERROR);
                return;
            }

            // 3. 检查LLC运行状态
            if (!LLC_IsRunning()) {
                System_TriggerFault(FAULT_MODULE_RUNTIME_ERROR);
                return;
            }

            // 4. 检查均流均压
            if (!LLC_IsBalanced()) {
                System_TriggerFault(FAULT_LLC_IMBALANCE);
                return;
            }

            // 5. 检查母线电压
            float bus_voltage = LLC_GetOutputVoltage();
            if (bus_voltage > 880.0f) {
                System_TriggerFault(FAULT_BUS_OVERVOLTAGE);
                return;
            } else if (bus_voltage < 720.0f) {
                System_TriggerFault(FAULT_BUS_UNDERVOLTAGE);
                return;
            }

            // 正常运行，无异常
            break;
        }

        /*--------------------------------------------------------------------
         * 放电停机（由STOPPING状态处理）
         *--------------------------------------------------------------------*/
        case DISCHARGE_STOPPING:
            // 此状态由State_Stopping_Handler处理
            break;

        default:
            // 未知子状态，触发故障
            System_TriggerFault(FAULT_MODULE_RUNTIME_ERROR);
            break;
    }
}

/**
 * @brief FAULT状态处理
 */
static void State_Fault_Handler(void)
{
    // 在故障状态下，确保所有模块已停止
    ShutdownAllModules();

    // 断开所有继电器和接触器
    HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_RESET);

    // 等待用户清除故障
    // 通过System_ClearFault()接口清除
}

/**
 * @brief STOPPING状态处理
 */
static void State_Stopping_Handler(void)
{
    uint32_t elapsed = GetElapsedTime(g_sys_ctx.state_entry_time);

    // 根据当前模式执行停机序列
    if (g_sys_ctx.mode == SYS_MODE_CHARGING) {
        /*--------------------------------------------------------------------
         * 充电模式停机序列：LLC停 → PFC停 → 断开继电器
         *--------------------------------------------------------------------*/
        // 步骤1: 停止LLC
        if (LLC_IsRunning()) {
            LLC_Stop();
        }

        // 步骤2: 等待LLC完全停止（延时200ms）
        if (elapsed >= 200 && PFC_IsRunning()) {
            PFC_Stop();
        }

        // 步骤3: 等待PFC完全停止（延时500ms）
        if (elapsed >= 700) {
            HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);

            // 停机完成，回到IDLE
            TransitionToState(SYS_STATE_IDLE);
        }
    }
    else if (g_sys_ctx.mode == SYS_MODE_DISCHARGING) {
        /*--------------------------------------------------------------------
         * 放电模式停机序列：逆变器停 → 断接触器 → LLC停
         *--------------------------------------------------------------------*/
        // 步骤1: 停止逆变器（功率降为0）
        if (GridInv_IsRunning()) {
            GridInv_Stop();
        }

        // 步骤2: 等待逆变器完全停止（延时500ms）
        if (elapsed >= 500) {
            HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_RESET);
        }

        // 步骤3: 断开接触器后等待200ms，然后停止LLC
        if (elapsed >= 700 && LLC_IsRunning()) {
            LLC_Stop();
        }

        // 步骤4: 等待LLC完全停止（延时1000ms）
        if (elapsed >= 1700) {
            // 停机完成，回到IDLE
            TransitionToState(SYS_STATE_IDLE);
        }
    }
    else {
        // 未知模式，直接关闭所有模块
        ShutdownAllModules();
        TransitionToState(SYS_STATE_IDLE);
    }

    // 超时保护
    if (elapsed > TIMEOUT_STOPPING_MS) {
        // 强制关闭所有模块
        ShutdownAllModules();
        TransitionToState(SYS_STATE_IDLE);
    }
}

/******************************************************************************
 * 外部接口函数实现
 ******************************************************************************/

/**
 * @brief 请求进入充电模式
 */
bool System_RequestCharging(void)
{
    // 只有在IDLE状态下才能接受充电请求
    if (g_sys_ctx.state != SYS_STATE_IDLE) {
        return false;
    }

    // 检查电网状态
    if (!GridMonitor_IsGridOK()) {
        System_TriggerFault(FAULT_GRID_ABNORMAL);
        return false;
    }

    // 切换到充电模式
    g_sys_ctx.mode = SYS_MODE_CHARGING;
    g_sys_ctx.charge_sub = CHARGE_PRECHARGE;
    TransitionToState(SYS_STATE_CHARGING);

    return true;
}

/**
 * @brief 请求进入放电模式
 */
bool System_RequestDischarging(void)
{
    // 只有在IDLE状态下才能接受放电请求
    if (g_sys_ctx.state != SYS_STATE_IDLE) {
        return false;
    }

    // 检查电网状态
    if (!GridMonitor_IsGridOK()) {
        System_TriggerFault(FAULT_GRID_ABNORMAL);
        return false;
    }

    // 切换到放电模式
    g_sys_ctx.mode = SYS_MODE_DISCHARGING;
    g_sys_ctx.discharge_sub = DISCHARGE_STARTING_LLC;
    TransitionToState(SYS_STATE_DISCHARGING);

    return true;
}

/**
 * @brief 请求停机
 */
bool System_RequestStop(void)
{
    // 如果已经在IDLE或STOPPING状态，拒绝请求
    if (g_sys_ctx.state == SYS_STATE_IDLE || g_sys_ctx.state == SYS_STATE_STOPPING) {
        return false;
    }

    // 如果在FAULT状态，直接回到IDLE（已经停止）
    if (g_sys_ctx.state == SYS_STATE_FAULT) {
        TransitionToState(SYS_STATE_IDLE);
        return true;
    }

    // 设置停机请求标志
    g_sys_ctx.stop_requested = true;

    return true;
}

/**
 * @brief 触发系统故障
 */
void System_TriggerFault(uint32_t fault_code)
{
    // 记录故障码（支持多个故障叠加）
    g_sys_ctx.fault_code |= fault_code;
    g_sys_ctx.fault_count++;

    // 立即切换到FAULT状态
    TransitionToState(SYS_STATE_FAULT);
}

/**
 * @brief 清除故障
 */
bool System_ClearFault(void)
{
    // 只有在FAULT状态下才能清除故障
    if (g_sys_ctx.state != SYS_STATE_FAULT) {
        return false;
    }

    // 清除故障码和标志
    g_sys_ctx.fault_code = FAULT_NONE;
    g_sys_ctx.emergency_stop = false;
    g_sys_ctx.stop_requested = false;

    // 回到IDLE状态
    TransitionToState(SYS_STATE_IDLE);

    return true;
}

/**
 * @brief 获取当前系统状态
 */
SystemState_t System_GetState(void)
{
    return g_sys_ctx.state;
}

/**
 * @brief 获取当前运行模式
 */
SystemMode_t System_GetMode(void)
{
    return g_sys_ctx.mode;
}

/**
 * @brief 获取当前故障码
 */
uint32_t System_GetFaultCode(void)
{
    return g_sys_ctx.fault_code;
}

/**
 * @brief 获取充电子状态
 */
ChargingSubState_t System_GetChargingSubState(void)
{
    return g_sys_ctx.charge_sub;
}

/**
 * @brief 获取放电子状态
 */
DischargingSubState_t System_GetDischargingSubState(void)
{
    return g_sys_ctx.discharge_sub;
}

/**
 * @brief 获取系统上下文指针
 */
const SystemContext_t* System_GetContext(void)
{
    return &g_sys_ctx;
}

/******************************************************************************
 * 内部辅助函数实现
 ******************************************************************************/

/**
 * @brief 状态转换函数
 * @param new_state 新状态
 */
static void TransitionToState(SystemState_t new_state)
{
    // 状态切换时的清理工作
    if (new_state != g_sys_ctx.state) {
        // 记录新状态进入时间
        g_sys_ctx.state_entry_time = HAL_GetTick();
        g_sys_ctx.substep_entry_time = HAL_GetTick();

        // 状态切换
        g_sys_ctx.state = new_state;

        // 根据新状态执行初始化
        if (new_state == SYS_STATE_IDLE) {
            g_sys_ctx.mode = SYS_MODE_NONE;
            g_sys_ctx.stop_requested = false;
            ResetSubStates();
        }
        else if (new_state == SYS_STATE_FAULT) {
            // 故障状态下，立即关闭所有模块
            ShutdownAllModules();
        }
    }
}

/**
 * @brief 重置所有子状态
 */
static void ResetSubStates(void)
{
    g_sys_ctx.charge_sub = CHARGE_IDLE;
    g_sys_ctx.discharge_sub = DISCHARGE_IDLE;
}

/**
 * @brief 计算从指定时间到现在的经过时间
 * @param start_time 起始时间戳
 * @return 经过的毫秒数
 */
static uint32_t GetElapsedTime(uint32_t start_time)
{
    uint32_t current_time = HAL_GetTick();

    // 处理时间戳溢出情况
    if (current_time >= start_time) {
        return current_time - start_time;
    } else {
        return (0xFFFFFFFF - start_time) + current_time + 1;
    }
}

/**
 * @brief 关闭所有功能模块
 */
static void ShutdownAllModules(void)
{
    // 停止所有模块
    if (PFC_IsRunning()) {
        PFC_Stop();
    }

    if (LLC_IsRunning()) {
        LLC_Stop();
    }

    if (GridInv_IsRunning()) {
        GridInv_Stop();
    }

    // 断开所有继电器和接触器
    HAL_GPIO_WritePin(GPIO_RELAY_PRECHARGE, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_RELAY_MAIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_GRID_CONTACTOR, GPIO_PIN_RESET);
}

/******************************************************************************
 * End of file
 ******************************************************************************/
