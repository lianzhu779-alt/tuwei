/******************************************************************************
 * @file    hal.c
 * @brief   硬件抽象层接口实现
 * @details 基于TI C2000 (TMS320F280049) 的 DriverLib 实现
 ******************************************************************************/

#include "hal.h"

// 尝试包含TI C2000标准头文件
// 如果编译报错，请确保工程包含路径正确设置
#include "device.h"
#include "driverlib.h"

/******************************************************************************
 * 私有变量
 ******************************************************************************/

static volatile uint32_t uwTick = 0;

/******************************************************************************
 * 内部函数声明
 ******************************************************************************/

__interrupt void HAL_Timer0_ISR(void);

/******************************************************************************
 * 接口函数实现
 ******************************************************************************/

/**
 * @brief GPIO初始化
 */
void HAL_GPIO_Init(uint32_t pin, GPIO_Mode_t mode) {
  // C2000 DriverLib GPIO配置
  GPIO_setMasterCore(pin, GPIO_CORE_CPU1);
  GPIO_setPinConfig(GPIO_0_GPIO0 +
                    (pin << 16)); // 简化的Pin Config映射，需根据实际调整

  if (mode == GPIO_MODE_OUTPUT) {
    GPIO_setDirectionMode(pin, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(pin, GPIO_PIN_TYPE_STD);
  } else {
    GPIO_setDirectionMode(pin, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(pin, GPIO_PIN_TYPE_STD); // 或 GPIO_PIN_TYPE_PULLUP
  }
}

/**
 * @brief 写GPIO电平
 */
void HAL_GPIO_WritePin(uint32_t pin, GPIO_PinState_t state) {
  GPIO_writePin(pin, state);
}

/**
 * @brief 读GPIO电平
 */
GPIO_PinState_t HAL_GPIO_ReadPin(uint32_t pin) {
  return (GPIO_PinState_t)GPIO_readPin(pin);
}

/**
 * @brief 获取系统时间戳
 * @return 系统运行时间（毫秒）
 */
uint32_t HAL_GetTick(void) { return uwTick; }

/**
 * @brief 延时函数
 * @param ms 延时时间（毫秒）
 */
void HAL_Delay(uint32_t ms) {
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = ms;

  while ((HAL_GetTick() - tickstart) < wait) {
  }
}

/******************************************************************************
 * 系统初始化与中断管理
 ******************************************************************************/

/**
 * @brief  系统底层初始化 (供 main 调用)
 * @details 配置CPU Timer0产生1ms中断作为系统Tick
 */
void HAL_System_Init(void) {
  // 停止定时器 (在配置前先停止是标准做法)
  CPUTimer_stopTimer(CPUTIMER0_BASE);

  // 配置定时器周期为 1ms
  // 注意：对于递减计数器，实际周期 = (Period + 1) / SYSCLK
  // 所以要减1以获得精确的1ms
  CPUTimer_setPeriod(CPUTIMER0_BASE, (DEVICE_SYSCLK_FREQ / 1000) - 1);

  // 配置预分频器 (0表示1:1分频，实际分频值 = prescaler + 1)
  CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

  // 设置仿真模式
  CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                            CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

  // 注册中断服务函数
  Interrupt_register(INT_TIMER0, &HAL_Timer0_ISR);

  // 使能定时器中断
  CPUTimer_enableInterrupt(CPUTIMER0_BASE);

  // 使能PIE中断
  Interrupt_enable(INT_TIMER0);

  // 启动定时器 (此函数会自动重载计数器并清除TSS位)
  CPUTimer_startTimer(CPUTIMER0_BASE);

  // 注意：确保在main()中已经调用了 EINT; 使能全局中断
}

/**
 * @brief  增加系统Tick计数 (被ISR调用)
 */
void HAL_IncTick(void) { uwTick++; }

/**
 * @brief  CPU Timer 0 中断服务函数
 */
__interrupt void HAL_Timer0_ISR(void) {
  HAL_IncTick();

  // 清除中断标志
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
