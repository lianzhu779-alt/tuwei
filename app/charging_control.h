/******************************************************************************
 * @file    charging_control.h
 * @brief   充电模式管理模块 - 三段式充电控制（CC-CV-Float）
 * @details 管理PFC+LLC充电过程，实现恒流、恒压、浮充三阶段充电
 *          充电路径：AC220V → PFC(800V) → LLC(降压) → DC125V → 电池
 * @author  Generated for TI DSP280049 Platform
 * @date    2025-12-26
 ******************************************************************************/

#ifndef CHARGING_CONTROL_H
#define CHARGING_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * 充电阶段枚举定义
 ******************************************************************************/

/**
 * @brief 充电阶段状态
 */
typedef enum {
    CHARGE_STAGE_IDLE = 0,        /**< 空闲状态 */
    CHARGE_STAGE_PRECHARGE,       /**< 预充电阶段（母线预充到350V） */
    CHARGE_STAGE_PFC_START,       /**< PFC启动阶段（升压到800V） */
    CHARGE_STAGE_LLC_START,       /**< LLC启动阶段（降压到125V） */
    CHARGE_STAGE_CC,              /**< 恒流充电阶段（Constant Current） */
    CHARGE_STAGE_CV,              /**< 恒压充电阶段（Constant Voltage） */
    CHARGE_STAGE_FLOAT,           /**< 浮充阶段（Float Charge） */
    CHARGE_STAGE_COMPLETE,        /**< 充电完成 */
    CHARGE_STAGE_ERROR            /**< 错误状态 */
} ChargeStage_t;

/******************************************************************************
 * 充电配置参数结构体
 ******************************************************************************/

/**
 * @brief 充电配置参数
 * @details 包含电池参数、充电参数、超时设置等
 */
typedef struct {
    /* 电池参数 */
    float battery_voltage_nom;    /**< 标称电压（V），默认125V */
    float battery_voltage_max;    /**< 最大充电电压（V），默认137.5V */
    float battery_voltage_min;    /**< 最小放电电压（V），默认100V */
    float battery_capacity_ah;    /**< 电池容量（Ah），用于SOC估算 */

    /* 充电参数 - CC阶段 */
    float charge_current_cc;      /**< 恒流充电电流（A），默认40A */

    /* 充电参数 - CV阶段 */
    float charge_voltage_cv;      /**< 恒压充电电压（V），默认137.5V */
    float charge_current_cutoff;  /**< CV截止电流（A），默认2A */

    /* 充电参数 - Float阶段 */
    float charge_voltage_float;   /**< 浮充电压（V），默认131.25V (125×1.05) */
    uint32_t float_time_ms;       /**< 浮充持续时间（ms），默认1小时 */

    /* 功率限制 */
    float charge_power_max;       /**< 最大充电功率（W），默认5000W */

    /* 超时设置 */
    uint32_t timeout_precharge;   /**< 预充电超时（ms），默认5秒 */
    uint32_t timeout_pfc_start;   /**< PFC启动超时（ms），默认5秒 */
    uint32_t timeout_llc_start;   /**< LLC启动超时（ms），默认3秒 */
    uint32_t timeout_charge_total;/**< 总充电超时（ms），默认4小时 */

    /* 保护阈值 */
    float protection_voltage_max; /**< 过压保护阈值（V），默认140V */
    float protection_current_max; /**< 过流保护阈值（A），默认50A */
    float protection_imbalance_v; /**< 电压不平衡阈值（V），默认80V */
    float protection_imbalance_i; /**< 电流不平衡阈值（A），默认10A */

} ChargeConfig_t;

/******************************************************************************
 * 充电运行状态结构体
 ******************************************************************************/

/**
 * @brief 充电运行状态
 * @details 包含当前阶段、实时数据、统计信息、故障标志等
 */
typedef struct {
    /* 状态标志 */
    ChargeStage_t stage;          /**< 当前充电阶段 */
    bool is_running;              /**< 运行标志 */

    /* 时间戳 */
    uint32_t stage_entry_time;    /**< 进入当前阶段的时间戳（ms） */
    uint32_t charge_start_time;   /**< 充电开始时间戳（ms） */

    /* 实时测量数据 */
    float battery_voltage;        /**< 电池电压（V） */
    float battery_current;        /**< 充电电流（A） */
    float battery_power;          /**< 充电功率（W） */
    float bus_voltage;            /**< 800V母线电压（V） */
    float grid_voltage;           /**< 电网电压（V） */
    float power_factor;           /**< 功率因数 */

    /* LLC均压均流状态 */
    float llc_vc1;                /**< LLC输入电容C1电压（V） */
    float llc_vc2;                /**< LLC输入电容C2电压（V） */
    float llc_iout1;              /**< LLC1输出电流（A） */
    float llc_iout2;              /**< LLC2输出电流（A） */

    /* 统计数据 */
    uint32_t charge_time_sec;     /**< 累计充电时间（秒） */
    float charge_ah;              /**< 累计充电电量（Ah） */
    float charge_wh;              /**< 累计充电能量（Wh） */
    float soc_estimate;           /**< SOC估算（%），0-100 */

    /* 故障信息 */
    uint32_t fault_flags;         /**< 故障位掩码 */
    uint32_t warning_flags;       /**< 警告位掩码 */

} ChargeStatus_t;

/******************************************************************************
 * 故障码定义（位掩码）
 ******************************************************************************/

#define CHARGE_FAULT_NONE            (0x00000000UL)  /**< 无故障 */
#define CHARGE_FAULT_GRID_FAIL       (1UL << 0)     /**< 电网故障 */
#define CHARGE_FAULT_PRECHARGE_TO    (1UL << 1)     /**< 预充电超时 */
#define CHARGE_FAULT_PFC_START_TO    (1UL << 2)     /**< PFC启动超时 */
#define CHARGE_FAULT_LLC_START_TO    (1UL << 3)     /**< LLC启动超时 */
#define CHARGE_FAULT_IMBALANCE_V     (1UL << 4)     /**< 电压不平衡故障 */
#define CHARGE_FAULT_IMBALANCE_I     (1UL << 5)     /**< 电流不平衡故障 */
#define CHARGE_FAULT_BATTERY_OV      (1UL << 6)     /**< 电池过压 */
#define CHARGE_FAULT_BATTERY_OC      (1UL << 7)     /**< 充电过流 */
#define CHARGE_FAULT_CHARGE_TIMEOUT  (1UL << 8)     /**< 充电总超时 */
#define CHARGE_FAULT_PFC_UNSTABLE    (1UL << 9)     /**< PFC输出不稳定 */
#define CHARGE_FAULT_BUS_OV          (1UL << 10)    /**< 母线过压 */
#define CHARGE_FAULT_BUS_UV          (1UL << 11)    /**< 母线欠压 */
#define CHARGE_FAULT_LLC_FAIL        (1UL << 12)    /**< LLC运行失败 */
#define CHARGE_FAULT_PFC_FAIL        (1UL << 13)    /**< PFC运行失败 */

/******************************************************************************
 * 警告码定义（位掩码）
 ******************************************************************************/

#define CHARGE_WARN_NONE             (0x00000000UL)  /**< 无警告 */
#define CHARGE_WARN_POWER_LIMIT      (1UL << 0)      /**< 功率限制激活 */
#define CHARGE_WARN_TEMP_HIGH        (1UL << 1)      /**< 温度偏高 */
#define CHARGE_WARN_IMBALANCE_MINOR  (1UL << 2)      /**< 轻微不平衡 */
#define CHARGE_WARN_SLOW_CHARGE      (1UL << 3)      /**< 充电速度慢 */

/******************************************************************************
 * 充电曲线参数默认值
 ******************************************************************************/

/* 电池参数默认值 */
#define DEFAULT_BATTERY_VOLTAGE_NOM     (125.0f)   /**< 标称电压 125V */
#define DEFAULT_BATTERY_VOLTAGE_MAX     (137.5f)   /**< 最大电压 137.5V */
#define DEFAULT_BATTERY_VOLTAGE_MIN     (100.0f)   /**< 最小电压 100V */
#define DEFAULT_BATTERY_CAPACITY_AH     (100.0f)   /**< 电池容量 100Ah */

/* 充电参数默认值 */
#define DEFAULT_CHARGE_CURRENT_CC       (40.0f)    /**< CC电流 40A */
#define DEFAULT_CHARGE_VOLTAGE_CV       (137.5f)   /**< CV电压 137.5V */
#define DEFAULT_CHARGE_CURRENT_CUTOFF   (2.0f)     /**< 截止电流 2A */
#define DEFAULT_CHARGE_VOLTAGE_FLOAT    (131.25f)  /**< 浮充电压 131.25V */
#define DEFAULT_CHARGE_POWER_MAX        (5000.0f)  /**< 最大功率 5kW */

/* 超时默认值 */
#define DEFAULT_TIMEOUT_PRECHARGE       (5000)     /**< 预充电超时 5s */
#define DEFAULT_TIMEOUT_PFC_START       (5000)     /**< PFC启动超时 5s */
#define DEFAULT_TIMEOUT_LLC_START       (3000)     /**< LLC启动超时 3s */
#define DEFAULT_TIMEOUT_CHARGE_TOTAL    (14400000) /**< 总充电超时 4h */
#define DEFAULT_FLOAT_TIME_MS           (3600000)  /**< 浮充时间 1h */

/* 保护阈值默认值 */
#define DEFAULT_PROTECTION_VOLTAGE_MAX  (140.0f)   /**< 过压保护 140V */
#define DEFAULT_PROTECTION_CURRENT_MAX  (50.0f)    /**< 过流保护 50A */
#define DEFAULT_PROTECTION_IMBALANCE_V  (80.0f)    /**< 电压不平衡 80V */
#define DEFAULT_PROTECTION_IMBALANCE_I  (10.0f)    /**< 电流不平衡 10A */

/* 预充电阈值 */
#define PRECHARGE_VOLTAGE_THRESHOLD     (350.0f)   /**< 预充电完成阈值 350V */

/* PFC稳定判断 */
#define PFC_VOLTAGE_TARGET              (800.0f)   /**< PFC目标电压 800V */
#define PFC_VOLTAGE_MIN                 (780.0f)   /**< PFC最低电压 780V */
#define PFC_VOLTAGE_MAX                 (820.0f)   /**< PFC最高电压 820V */

/* LLC启动判断 */
#define LLC_OUTPUT_VOLTAGE_MIN          (100.0f)   /**< LLC输出最低电压 100V */
#define LLC_OUTPUT_VOLTAGE_MAX          (140.0f)   /**< LLC输出最高电压 140V */
#define LLC_INPUT_IMBALANCE_MAX         (50.0f)    /**< LLC启动时最大不平衡 50V */

/******************************************************************************
 * 外部接口函数声明
 ******************************************************************************/

/**
 * @brief 充电模块初始化
 * @details 初始化充电控制模块，设置默认配置参数
 */
void Charging_Init(void);

/**
 * @brief 启动充电
 * @param config 充电配置参数（NULL则使用默认配置）
 * @return true-启动成功, false-启动失败
 */
bool Charging_Start(const ChargeConfig_t* config);

/**
 * @brief 正常停止充电
 * @details 按顺序停止LLC、PFC，断开继电器
 */
void Charging_Stop(void);

/**
 * @brief 紧急停止充电
 * @details 立即停止所有模块，用于紧急情况
 */
void Charging_Abort(void);

/**
 * @brief 充电控制任务 - 1kHz调用
 * @details 状态机主循环，阶段管理，保护检查
 */
void Charging_Run_1kHz(void);

/**
 * @brief 充电统计任务 - 100Hz调用
 * @details 更新充电时间、电量、能量统计
 */
void Charging_Run_100Hz(void);

/**
 * @brief 检查充电是否运行中
 * @return true-运行中, false-已停止
 */
bool Charging_IsRunning(void);

/**
 * @brief 获取当前充电阶段
 * @return 当前充电阶段枚举
 */
ChargeStage_t Charging_GetStage(void);

/**
 * @brief 获取充电状态指针
 * @return 充电状态结构体指针（只读）
 */
const ChargeStatus_t* Charging_GetStatus(void);

/**
 * @brief 获取充电进度
 * @details 基于SOC估算或充电时间计算进度
 * @return 充电进度百分比（0-100）
 */
float Charging_GetProgress(void);

/**
 * @brief 设置充电电流限制
 * @param i_max 最大充电电流（A）
 */
void Charging_SetCurrentLimit(float i_max);

/**
 * @brief 设置充电电压限制
 * @param v_max 最大充电电压（V）
 */
void Charging_SetVoltageLimit(float v_max);

/**
 * @brief 设置充电功率限制
 * @param p_max 最大充电功率（W）
 */
void Charging_SetPowerLimit(float p_max);

/**
 * @brief 清除故障标志（需要先停止充电）
 * @return true-清除成功, false-清除失败
 */
bool Charging_ClearFaults(void);

/**
 * @brief 获取充电阶段名称字符串
 * @param stage 充电阶段枚举
 * @return 阶段名称字符串
 */
const char* Charging_GetStageName(ChargeStage_t stage);

/**
 * @brief 获取故障描述字符串
 * @param fault_code 故障码
 * @return 故障描述字符串
 */
const char* Charging_GetFaultString(uint32_t fault_code);

#endif /* CHARGING_CONTROL_H */
