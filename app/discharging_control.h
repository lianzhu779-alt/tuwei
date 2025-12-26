/******************************************************************************
 * @file    discharging_control.h
 * @brief   放电模式管理模块 - 并网逆变器控制
 * @details 管理LLC升压+三相并网逆变器，实现安全并网和功率输出
 *          放电路径：电池125V → LLC(升压) → 800V → 逆变器 → 电网
 * @author  Generated for TI DSP280049 Platform
 * @date    2025-12-26
 ******************************************************************************/

#ifndef DISCHARGING_CONTROL_H
#define DISCHARGING_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 放电阶段枚举定义
 ******************************************************************************/

/**
 * @brief 放电阶段状态
 */
typedef enum {
    DISCHARGE_STAGE_IDLE = 0,         /**< 空闲状态 */
    DISCHARGE_STAGE_LLC_START,        /**< LLC升压启动（125V→800V） */
    DISCHARGE_STAGE_DC_BUS_STABLE,    /**< 800V母线稳定检查 */
    DISCHARGE_STAGE_GRID_SYNCING,     /**< 电网同步中（PLL锁相） */
    DISCHARGE_STAGE_GRID_CONNECTING,  /**< 并网投入（闭合接触器） */
    DISCHARGE_STAGE_POWER_RAMPING,    /**< 功率爬坡阶段 */
    DISCHARGE_STAGE_RUNNING,          /**< 正常运行（并网发电） */
    DISCHARGE_STAGE_COMPLETE,         /**< 放电完成 */
    DISCHARGE_STAGE_ERROR             /**< 错误状态 */
} DischargeStage_t;

/******************************************************************************
 * 放电配置参数结构体
 ******************************************************************************/

/**
 * @brief 放电配置参数
 * @details 包含电池参数、放电参数、并网参数、同步参数等
 */
typedef struct {
    /* 电池参数 */
    float battery_voltage_nom;        /**< 标称电压（V），默认125V */
    float battery_voltage_min;        /**< 最小电压（V），默认100V（放电截止） */
    float battery_voltage_max;        /**< 最大电压（V），默认137.5V */
    float battery_capacity_ah;        /**< 电池容量（Ah），用于SOC估算 */

    /* 放电参数 */
    float discharge_power_target;     /**< 目标放电功率（W），默认3000W */
    float discharge_power_max;        /**< 最大放电功率（W），默认5000W */
    float discharge_current_max;      /**< 最大放电电流（A），默认50A */

    /* 并网参数 */
    float dc_bus_voltage_ref;         /**< 直流母线电压参考（V），默认800V */
    float reactive_power_ref;         /**< 无功功率参考（Var），默认0（纯有功） */
    float power_ramp_rate;            /**< 功率爬坡速率（W/s），默认500W/s */
    float power_ramp_down_rate;       /**< 功率下降速率（W/s），默认1000W/s */

    /* 电网同步参数（参考GB/T 19939和IEEE 1547） */
    float sync_voltage_tolerance;     /**< 电压同步容差（V），默认±10V */
    float sync_freq_tolerance;        /**< 频率同步容差（Hz），默认±0.2Hz */
    float sync_phase_tolerance;       /**< 相位同步容差（度），默认±5° */
    uint32_t sync_hold_time;          /**< 同步保持时间（ms），默认500ms */

    /* 超时设置 */
    uint32_t timeout_llc_start;       /**< LLC启动超时（ms），默认3秒 */
    uint32_t timeout_dc_bus_stable;   /**< 母线稳定超时（ms），默认3秒 */
    uint32_t timeout_grid_sync;       /**< 电网同步超时（ms），默认5秒 */
    uint32_t timeout_grid_connect;    /**< 并网投入超时（ms），默认2秒 */
    uint32_t timeout_power_ramp;      /**< 功率爬坡超时（ms），默认10秒 */
    uint32_t timeout_discharge_total; /**< 总放电超时（ms），默认4小时 */

    /* 保护阈值 */
    float protection_current_max;     /**< 输出过流保护（A RMS），默认30A */
    float protection_dcbus_ov;        /**< 母线过压保护（V），默认850V */
    float protection_dcbus_uv;        /**< 母线欠压保护（V），默认750V */
    float protection_imbalance_i;     /**< 电流不平衡保护（A），默认10A */
    float protection_imbalance_v;     /**< 电压不平衡保护（V），默认80V */

} DischargeConfig_t;

/******************************************************************************
 * 放电运行状态结构体
 ******************************************************************************/

/**
 * @brief 放电运行状态
 * @details 包含当前阶段、实时数据、统计信息、故障标志等
 */
typedef struct {
    /* 状态标志 */
    DischargeStage_t stage;           /**< 当前放电阶段 */
    bool is_running;                  /**< 运行标志 */
    bool grid_connected;              /**< 并网状态标志 */

    /* 时间戳 */
    uint32_t stage_entry_time;        /**< 进入当前阶段的时间戳（ms） */
    uint32_t discharge_start_time;    /**< 放电开始时间戳（ms） */
    uint32_t sync_start_time;         /**< 同步开始时间戳（ms） */
    uint32_t grid_connect_time;       /**< 并网时间戳（ms） */

    /* 电池侧实时数据 */
    float battery_voltage;            /**< 电池电压（V） */
    float battery_current;            /**< 放电电流（A） */
    float battery_power;              /**< 电池功率（W） */

    /* LLC状态 */
    float dc_bus_voltage;             /**< 800V母线电压（V） */
    float llc_iin1;                   /**< LLC1输入电流（A） */
    float llc_iin2;                   /**< LLC2输入电流（A） */
    float llc_vout1;                  /**< LLC1输出电压（V，目标400V） */
    float llc_vout2;                  /**< LLC2输出电压（V，目标400V） */

    /* 电网侧实时数据 */
    float grid_voltage;               /**< 电网电压（V RMS） */
    float grid_frequency;             /**< 电网频率（Hz） */
    float phase_difference;           /**< 相位差（度） */

    /* 逆变器输出 */
    float active_power;               /**< 实际有功功率（W） */
    float reactive_power;             /**< 实际无功功率（Var） */
    float output_current;             /**< 输出电流（A RMS） */
    float power_factor;               /**< 功率因数 */

    /* 统计数据 */
    uint32_t discharge_time_sec;      /**< 累计放电时间（秒） */
    float discharge_ah;               /**< 累计放电电量（Ah） */
    float discharge_wh;               /**< 累计放电能量（Wh，从电池） */
    float total_energy_export;        /**< 累计输出能量（Wh，到电网） */
    float efficiency;                 /**< 放电效率（%） */
    float soc_estimate;               /**< SOC估算（%），0-100 */

    /* 并网质量统计 */
    uint32_t sync_loss_count;         /**< 同步丢失次数 */
    uint32_t grid_fault_count;        /**< 电网故障次数 */
    uint32_t islanding_detect_count;  /**< 孤岛检测次数 */
    uint32_t total_grid_time_sec;     /**< 累计并网时间（秒） */

    /* 故障信息 */
    uint32_t fault_flags;             /**< 故障位掩码 */
    uint32_t warning_flags;           /**< 警告位掩码 */

} DischargeStatus_t;

/******************************************************************************
 * 故障码定义（位掩码）
 ******************************************************************************/

#define DISCHARGE_FAULT_NONE            (0x00000000UL)  /**< 无故障 */
#define DISCHARGE_FAULT_BATTERY_UV      (1UL << 0)      /**< 电池欠压 */
#define DISCHARGE_FAULT_BATTERY_OC      (1UL << 1)      /**< 电池过流 */
#define DISCHARGE_FAULT_LLC_START_TO    (1UL << 2)      /**< LLC启动超时 */
#define DISCHARGE_FAULT_LLC_IMBALANCE   (1UL << 3)      /**< LLC不平衡 */
#define DISCHARGE_FAULT_DCBUS_OV        (1UL << 4)      /**< 母线过压 */
#define DISCHARGE_FAULT_DCBUS_UV        (1UL << 5)      /**< 母线欠压 */
#define DISCHARGE_FAULT_DCBUS_UNSTABLE  (1UL << 6)      /**< 母线不稳定 */
#define DISCHARGE_FAULT_GRID_SYNC_TO    (1UL << 7)      /**< 电网同步超时 */
#define DISCHARGE_FAULT_GRID_SYNC_LOST  (1UL << 8)      /**< 同步丢失 */
#define DISCHARGE_FAULT_GRID_VOLTAGE    (1UL << 9)      /**< 电网电压异常 */
#define DISCHARGE_FAULT_GRID_FREQUENCY  (1UL << 10)     /**< 电网频率异常 */
#define DISCHARGE_FAULT_ISLANDING       (1UL << 11)     /**< 检测到孤岛 */
#define DISCHARGE_FAULT_OVERCURRENT     (1UL << 12)     /**< 输出过流 */
#define DISCHARGE_FAULT_DISCHARGE_TO    (1UL << 13)     /**< 放电超时 */
#define DISCHARGE_FAULT_INVERTER_FAIL   (1UL << 14)     /**< 逆变器故障 */
#define DISCHARGE_FAULT_LLC_FAIL        (1UL << 15)     /**< LLC运行故障 */

/******************************************************************************
 * 警告码定义（位掩码）
 ******************************************************************************/

#define DISCHARGE_WARN_NONE             (0x00000000UL)  /**< 无警告 */
#define DISCHARGE_WARN_POWER_LIMIT      (1UL << 0)      /**< 功率限制激活 */
#define DISCHARGE_WARN_BATTERY_LOW      (1UL << 1)      /**< 电池电量低 */
#define DISCHARGE_WARN_IMBALANCE_MINOR  (1UL << 2)      /**< 轻微不平衡 */
#define DISCHARGE_WARN_GRID_QUALITY     (1UL << 3)      /**< 电网质量差 */
#define DISCHARGE_WARN_TEMP_HIGH        (1UL << 4)      /**< 温度偏高 */

/******************************************************************************
 * 放电参数默认值
 ******************************************************************************/

/* 电池参数默认值 */
#define DEFAULT_BATTERY_VOLTAGE_NOM_DIS     (125.0f)   /**< 标称电压 125V */
#define DEFAULT_BATTERY_VOLTAGE_MIN_DIS     (100.0f)   /**< 最小电压 100V */
#define DEFAULT_BATTERY_VOLTAGE_MAX_DIS     (137.5f)   /**< 最大电压 137.5V */
#define DEFAULT_BATTERY_CAPACITY_AH_DIS     (100.0f)   /**< 电池容量 100Ah */

/* 放电参数默认值 */
#define DEFAULT_DISCHARGE_POWER_TARGET      (3000.0f)  /**< 目标功率 3kW */
#define DEFAULT_DISCHARGE_POWER_MAX         (5000.0f)  /**< 最大功率 5kW */
#define DEFAULT_DISCHARGE_CURRENT_MAX       (50.0f)    /**< 最大电流 50A */

/* 并网参数默认值 */
#define DEFAULT_DC_BUS_VOLTAGE_REF          (800.0f)   /**< 母线电压 800V */
#define DEFAULT_REACTIVE_POWER_REF          (0.0f)     /**< 无功功率 0Var */
#define DEFAULT_POWER_RAMP_RATE             (500.0f)   /**< 爬坡速率 500W/s */
#define DEFAULT_POWER_RAMP_DOWN_RATE        (1000.0f)  /**< 下降速率 1000W/s */

/* 同步参数默认值（GB/T 19939标准） */
#define DEFAULT_SYNC_VOLTAGE_TOLERANCE      (10.0f)    /**< 电压容差 ±10V */
#define DEFAULT_SYNC_FREQ_TOLERANCE         (0.2f)     /**< 频率容差 ±0.2Hz */
#define DEFAULT_SYNC_PHASE_TOLERANCE        (5.0f)     /**< 相位容差 ±5° */
#define DEFAULT_SYNC_HOLD_TIME              (500)      /**< 保持时间 500ms */

/* 超时默认值 */
#define DEFAULT_TIMEOUT_LLC_START_DIS       (3000)     /**< LLC启动超时 3s */
#define DEFAULT_TIMEOUT_DC_BUS_STABLE       (3000)     /**< 母线稳定超时 3s */
#define DEFAULT_TIMEOUT_GRID_SYNC           (5000)     /**< 电网同步超时 5s */
#define DEFAULT_TIMEOUT_GRID_CONNECT        (2000)     /**< 并网投入超时 2s */
#define DEFAULT_TIMEOUT_POWER_RAMP          (10000)    /**< 功率爬坡超时 10s */
#define DEFAULT_TIMEOUT_DISCHARGE_TOTAL     (14400000) /**< 总放电超时 4h */

/* 保护阈值默认值 */
#define DEFAULT_PROTECTION_CURRENT_MAX      (30.0f)    /**< 过流保护 30A */
#define DEFAULT_PROTECTION_DCBUS_OV         (850.0f)   /**< 母线过压 850V */
#define DEFAULT_PROTECTION_DCBUS_UV         (750.0f)   /**< 母线欠压 750V */
#define DEFAULT_PROTECTION_IMBALANCE_I      (10.0f)    /**< 电流不平衡 10A */
#define DEFAULT_PROTECTION_IMBALANCE_V      (80.0f)    /**< 电压不平衡 80V */

/* 母线电压判断阈值 */
#define DCBUS_VOLTAGE_STABLE_MIN            (780.0f)   /**< 母线稳定最低电压 780V */
#define DCBUS_VOLTAGE_STABLE_MAX            (820.0f)   /**< 母线稳定最高电压 820V */
#define DCBUS_STABLE_TIME_MS                (500)      /**< 母线稳定持续时间 500ms */

/* 电网同步判断（参考标准值） */
#define GRID_VOLTAGE_NOMINAL                (220.0f)   /**< 标称电网电压 220V（线电压） */
#define GRID_FREQUENCY_NOMINAL              (50.0f)    /**< 标称电网频率 50Hz */

/******************************************************************************
 * 外部接口函数声明
 ******************************************************************************/

/**
 * @brief 放电模块初始化
 * @details 初始化放电控制模块，设置默认配置参数，初始化防孤岛保护
 */
void Discharging_Init(void);

/**
 * @brief 启动放电
 * @param config 放电配置参数（NULL则使用默认配置）
 * @return true-启动成功, false-启动失败
 */
bool Discharging_Start(const DischargeConfig_t* config);

/**
 * @brief 正常停止放电
 * @details 按顺序：降功率→断接触器→停逆变器→停LLC
 */
void Discharging_Stop(void);

/**
 * @brief 紧急停止放电（立即脱网）
 * @details 用于紧急情况，立即断开接触器和停止所有模块
 */
void Discharging_Abort(void);

/**
 * @brief 放电控制任务 - 1kHz调用
 * @details 状态机主循环，阶段管理，实时保护检查
 */
void Discharging_Run_1kHz(void);

/**
 * @brief 放电监测任务 - 100Hz调用
 * @details 电网监测、防孤岛检测、统计更新
 */
void Discharging_Run_100Hz(void);

/**
 * @brief 检查放电是否运行中
 * @return true-运行中, false-已停止
 */
bool Discharging_IsRunning(void);

/**
 * @brief 检查是否已并网
 * @return true-已并网, false-未并网
 */
bool Discharging_IsGridConnected(void);

/**
 * @brief 获取当前放电阶段
 * @return 当前放电阶段枚举
 */
DischargeStage_t Discharging_GetStage(void);

/**
 * @brief 获取放电状态指针
 * @return 放电状态结构体指针（只读）
 */
const DischargeStatus_t* Discharging_GetStatus(void);

/**
 * @brief 获取放电进度
 * @details 基于SOC估算或时间计算进度
 * @return 放电进度百分比（0-100）
 */
float Discharging_GetProgress(void);

/**
 * @brief 设置目标输出功率
 * @param p_target 目标有功功率（W）
 */
void Discharging_SetPowerTarget(float p_target);

/**
 * @brief 设置无功功率参考
 * @param q_target 无功功率（Var）
 */
void Discharging_SetReactivePower(float q_target);

/**
 * @brief 设置功率限制
 * @param p_max 最大功率（W）
 */
void Discharging_SetPowerLimit(float p_max);

/**
 * @brief 设置功率爬坡速率
 * @param rate 爬坡速率（W/s）
 */
void Discharging_SetRampRate(float rate);

/**
 * @brief 清除故障标志（需要先停止放电）
 * @return true-清除成功, false-清除失败
 */
bool Discharging_ClearFaults(void);

/**
 * @brief 获取放电阶段名称字符串
 * @param stage 放电阶段枚举
 * @return 阶段名称字符串
 */
const char* Discharging_GetStageName(DischargeStage_t stage);

/**
 * @brief 获取故障描述字符串
 * @param fault_code 故障码
 * @return 故障描述字符串
 */
const char* Discharging_GetFaultString(uint32_t fault_code);

#endif /* DISCHARGING_CONTROL_H */
