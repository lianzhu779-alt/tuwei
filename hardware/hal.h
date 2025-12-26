/******************************************************************************
 * @file    hal.h
 * @brief   硬件抽象层接口定义
 * @details 提供GPIO、时间戳等底层硬件接口
 ******************************************************************************/

#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * GPIO定义
 ******************************************************************************/

// GPIO引脚定义（需根据实际硬件配置）
#define GPIO_RELAY_PRECHARGE    0    /**< 预充电继电器GPIO */
#define GPIO_RELAY_MAIN         1    /**< 主继电器GPIO */
#define GPIO_GRID_CONTACTOR     2    /**< 并网接触器GPIO */

// GPIO模式
typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1
} GPIO_Mode_t;

// GPIO电平
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState_t;

/******************************************************************************
 * 接口函数声明
 ******************************************************************************/

/**
 * @brief GPIO初始化
 * @param pin GPIO引脚编号
 * @param mode GPIO模式（输入/输出）
 */
void HAL_GPIO_Init(uint32_t pin, GPIO_Mode_t mode);

/**
 * @brief 写GPIO电平
 * @param pin GPIO引脚编号
 * @param state GPIO电平状态
 */
void HAL_GPIO_WritePin(uint32_t pin, GPIO_PinState_t state);

/**
 * @brief 读GPIO电平
 * @param pin GPIO引脚编号
 * @return GPIO电平状态
 */
GPIO_PinState_t HAL_GPIO_ReadPin(uint32_t pin);

/**
 * @brief 获取系统时间戳
 * @return 系统运行时间（毫秒）
 */
uint32_t HAL_GetTick(void);

/**
 * @brief 延时函数
 * @param ms 延时时间（毫秒）
 */
void HAL_Delay(uint32_t ms);

/**
 * @brief 系统底层初始化
 * @details 配置CPU Timer0产生1ms中断作为系统Tick
 */
void HAL_System_Init(void);

#endif /* HAL_H */
