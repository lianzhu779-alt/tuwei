/******************************************************************************
 * @file    system_manager.h
 * @brief   顶层系统状态机管理 - PFC + 双交错LLC(IPOS) + 并网逆变器系统
 * @details 管理充电模式（AC→PFC→LLC→电池）和放电模式（电池→LLC→逆变器→电网）
 * @author  Generated for TI DSP280049 Platform
 * @date    2025-12-26
 ******************************************************************************/

#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 系统状态枚举定义
 ******************************************************************************/

/**
 * @brief 顶层系统状态
 */
typedef enum {
    SYS_STATE_IDLE = 0,           /**< 系统空闲，准备接受命令 */
    SYS_STATE_CHARGING,           /**< 充电模式运行中 */
    SYS_STATE_DISCHARGING,        /**< 放电模式运行中 */
    SYS_STATE_FAULT,              /**< 故障状态，需要清除后才能重启 */
    SYS_STATE_STOPPING            /**< 停机过渡状态 */
} SystemState_t;

/**
 * @brief 系统运行模式
 */
typedef enum {
    SYS_MODE_NONE = 0,            /**< 无模式（空闲或故障） */
    SYS_MODE_CHARGING,            /**< 充电模式：电网→电池 */
    SYS_MODE_DISCHARGING          /**< 放电模式：电池→电网 */
} SystemMode_t;

/**
 * @brief 充电模式子状态
 */
typedef enum {
    CHARGE_IDLE = 0,              /**< 充电空闲 */
    CHARGE_PRECHARGE,             /**< 预充电阶段 */
    CHARGE_STARTING_PFC,          /**< 启动PFC到800V */
    CHARGE_STARTING_LLC,          /**< 启动LLC降压到125V */
    CHARGE_RUNNING,               /**< 充电正常运行 */
    CHARGE_STOPPING               /**< 充电停机中 */
} ChargingSubState_t;

/**
 * @brief 放电模式子状态
 */
typedef enum {
    DISCHARGE_IDLE = 0,           /**< 放电空闲 */
    DISCHARGE_STARTING_LLC,       /**< 启动LLC升压到800V */
    DISCHARGE_GRID_SYNCING,       /**< 逆变器PLL同步 */
    DISCHARGE_GRID_CONNECTING,    /**< 并网投入阶段 */
    DISCHARGE_RUNNING,            /**< 放电正常运行 */
    DISCHARGE_STOPPING            /**< 放电停机中 */
} DischargingSubState_t;

/******************************************************************************
 * 故障码定义（位掩码）
 ******************************************************************************/

#define FAULT_NONE                  (0x00000000UL)  /**< 无故障 */
#define FAULT_GRID_ABNORMAL         (1UL << 0)     /**< 电网异常 */
#define FAULT_PFC_INIT_FAIL         (1UL << 1)     /**< PFC初始化失败 */
#define FAULT_PFC_START_TIMEOUT     (1UL << 2)     /**< PFC启动超时 */
#define FAULT_PFC_OUTPUT_UNSTABLE   (1UL << 3)     /**< PFC输出不稳定 */
#define FAULT_LLC_INIT_FAIL         (1UL << 4)     /**< LLC初始化失败 */
#define FAULT_LLC_START_TIMEOUT     (1UL << 5)     /**< LLC启动超时 */
#define FAULT_LLC_IMBALANCE         (1UL << 6)     /**< LLC均压均流失败 */
#define FAULT_GRID_INV_INIT_FAIL    (1UL << 7)     /**< 逆变器初始化失败 */
#define FAULT_GRID_SYNC_TIMEOUT     (1UL << 8)     /**< 电网同步超时 */
#define FAULT_GRID_LOST             (1UL << 9)     /**< 运行中电网丢失 */
#define FAULT_PRECHARGE_TIMEOUT     (1UL << 10)    /**< 预充电超时 */
#define FAULT_BUS_OVERVOLTAGE       (1UL << 11)    /**< 母线过压 */
#define FAULT_BUS_UNDERVOLTAGE      (1UL << 12)    /**< 母线欠压 */
#define FAULT_BATTERY_VOLTAGE_LOW   (1UL << 13)    /**< 电池电压过低 */
#define FAULT_EMERGENCY_STOP        (1UL << 14)    /**< 紧急停机 */
#define FAULT_MODULE_RUNTIME_ERROR  (1UL << 15)    /**< 模块运行时错误 */

/******************************************************************************
 * 时序参数定义（单位：ms，基于1kHz调用）
 ******************************************************************************/

#define TIMEOUT_PRECHARGE_MS        (2000)        /**< 预充电超时 2s */
#define TIMEOUT_PFC_START_MS        (5000)        /**< PFC启动超时 5s */
#define TIMEOUT_LLC_START_MS        (3000)        /**< LLC启动超时 3s */
#define TIMEOUT_GRID_SYNC_MS        (5000)        /**< 电网同步超时 5s */
#define TIMEOUT_GRID_CONNECT_MS     (5000)        /**< 并网投入超时 5s */
#define TIMEOUT_STOPPING_MS         (3000)        /**< 停机超时 3s */

#define DELAY_GRID_SYNC_STABLE_MS   (500)         /**< PLL锁定后稳定时间 500ms */
#define DELAY_MAIN_RELAY_MS         (100)         /**< 主继电器延时 100ms */

#define VOLTAGE_PRECHARGE_THRESHOLD (350.0f)      /**< 预充电电压阈值 350V */
#define VOLTAGE_BATTERY_MIN         (100.0f)      /**< 最低电池电压 100V */

/******************************************************************************
 * 系统上下文结构体
 ******************************************************************************/

/**
 * @brief 系统状态机上下文
 */
typedef struct {
    /* 状态变量 */
    SystemState_t state;                /**< 当前顶层状态 */
    SystemMode_t mode;                  /**< 当前运行模式 */
    ChargingSubState_t charge_sub;      /**< 充电子状态 */
    DischargingSubState_t discharge_sub;/**< 放电子状态 */

    /* 时间戳 */
    uint32_t state_entry_time;          /**< 当前状态进入时间戳（ms） */
    uint32_t substep_entry_time;        /**< 子步骤进入时间戳（ms） */

    /* 故障管理 */
    uint32_t fault_code;                /**< 故障码（位掩码） */
    bool emergency_stop;                /**< 紧急停机标志 */

    /* 运行标志 */
    bool stop_requested;                /**< 停机请求标志 */

    /* 统计信息（可选，用于调试） */
    uint32_t charge_count;              /**< 充电次数 */
    uint32_t discharge_count;           /**< 放电次数 */
    uint32_t fault_count;               /**< 故障次数 */

} SystemContext_t;

/******************************************************************************
 * 外部接口函数声明
 ******************************************************************************/

/**
 * @brief 系统初始化
 * @details 初始化所有模块（PFC, LLC, GridInv）和状态机上下文
 * @return true-成功, false-失败
 */
void System_Init(void);

/**
 * @brief 主状态机周期调用函数
 * @details 应在1kHz中断或任务中调用
 */
void System_StateMachine_1kHz(void);

/**
 * @brief 请求进入充电模式
 * @return true-请求接受, false-请求拒绝（当前状态不允许）
 */
bool System_RequestCharging(void);

/**
 * @brief 请求进入放电模式
 * @return true-请求接受, false-请求拒绝（当前状态不允许）
 */
bool System_RequestDischarging(void);

/**
 * @brief 请求停机
 * @return true-请求接受, false-已经在停机或空闲
 */
bool System_RequestStop(void);

/**
 * @brief 触发系统故障
 * @param fault_code 故障码（可以是多个故障的或值）
 */
void System_TriggerFault(uint32_t fault_code);

/**
 * @brief 清除故障（仅在FAULT状态下有效）
 * @details 清除故障后系统回到IDLE，需要重新发起模式请求
 * @return true-清除成功, false-清除失败
 */
bool System_ClearFault(void);

/**
 * @brief 获取当前系统状态
 * @return 当前顶层状态
 */
SystemState_t System_GetState(void);

/**
 * @brief 获取当前运行模式
 * @return 当前运行模式
 */
SystemMode_t System_GetMode(void);

/**
 * @brief 获取当前故障码
 * @return 故障码（位掩码）
 */
uint32_t System_GetFaultCode(void);

/**
 * @brief 获取充电子状态
 * @return 充电子状态
 */
ChargingSubState_t System_GetChargingSubState(void);

/**
 * @brief 获取放电子状态
 * @return 放电子状态
 */
DischargingSubState_t System_GetDischargingSubState(void);

/**
 * @brief 获取系统上下文指针（用于调试监控）
 * @return 系统上下文指针（只读）
 */
const SystemContext_t* System_GetContext(void);

#endif /* SYSTEM_MANAGER_H */
