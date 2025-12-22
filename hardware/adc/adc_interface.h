#ifndef ADC_INTERFACE_H_
#define ADC_INTERFACE_H_

#include "driverlib.h"
#include "device.h"
#include "board.h"

//
// ADC转换系数 - 预计算避免除法运算
//
#define ADC_TO_VOLT         (0.000732421875f)   // 3.0 / 4096

// 线电压: (ADC - Vref) * ADC_TO_VOLT * 507.61421
#define V_LL_SCALE          (0.371793469f)      // 3.0/4096 * 507.61421

// 直流电流: (ADC * ADC_TO_VOLT - 1.5) * 75.996431
// 展开为: ADC * K1 - K2
#define I_DC800_K1          (0.055669345f)      // 3.0/4096 * 75.996431
#define I_DC800_K2          (113.9946465f)      // 1.5 * 75.996431

// DC800V电容电压: ADC * ADC_TO_VOLT * 185.391175
#define V_DC800_CAP_SCALE   (0.135788086f)      // 3.0/4096 * 185.391175

// DC125V电压: ADC * ADC_TO_VOLT * 108.831692
#define V_DC125_SCALE       (0.079699707f)      // 3.0/4096 * 108.831692

// 三相电流: ((ADC * ADC_TO_VOLT + offset) * 1.7435897 - 2.5) * 32.467532
// 展开为: ADC * K1 - K2
#define I_PHASE_K1          (0.041459347f)      // 3.0/4096 * 1.7435897 * 32.467532
#define I_PHASE_A_K2        (73.246569f)        // (2.5 - 0.14 * 1.7435897) * 32.467532
#define I_PHASE_B_K2        (73.541344f)        // (2.5 - 0.135 * 1.7435897) * 32.467532
#define I_PHASE_C_K2        (73.823908f)        // (2.5 - 0.13 * 1.7435897) * 32.467532

//
// ADC原始数据结构 (寄存器值)
//
typedef struct {
    // ADCA - 线电压和温度传感器
    uint16_t V_LL_AB;
    uint16_t V_LL_BC;
    uint16_t V_LL_CA;
    uint16_t V_Ref_1V5;
    uint16_t T_LLC_Pri_MOS;
    uint16_t T_LLC_Sec_MOS;
    uint16_t T_Inv_3Ph_MOS;
    uint16_t T_Ambient;

    // ADCB - 直流母线测量
    uint16_t I_DC800;
    uint16_t V_DC800_CapU;
    uint16_t V_DC800_CapL;
    uint16_t V_DC125;

    // ADCC - 三相电流
    uint16_t I_Phase_A;
    uint16_t I_Phase_B;
    uint16_t I_Phase_C;
} ADC_SampleData_t;

//
// ADC物理值数据结构 (转换为实际单位)
//
typedef struct {
    // 线电压 (V RMS)
    float V_LL_AB;
    float V_LL_BC;
    float V_LL_CA;
    float V_Ref_1V5;

    // 温度 (摄氏度)
    float T_LLC_Pri_MOS;
    float T_LLC_Sec_MOS;
    float T_Inv_3Ph_MOS;
    float T_Ambient;

    // 直流母线测量
    float I_DC800;          // 直流电流 (A)
    float V_DC800_CapU;     // 上电容电压 (V)
    float V_DC800_CapL;     // 下电容电压 (V)
    float V_DC800_Total;    // 总母线电压 (V)
    float V_DC125;          // 125V母线电压 (V)

    // 三相电流 (A)
    float I_Phase_A;
    float I_Phase_B;
    float I_Phase_C;
} ADC_PhysicalData_t;

//
// 全局变量声明
//
extern ADC_SampleData_t g_ADC_Data;
extern ADC_PhysicalData_t g_ADC_PhysicalData;

//
// 函数声明
//
void ADC_Interface_Init(void);
void ADC_ReadAllResults(ADC_SampleData_t *data);
void ADC_ConvertToPhysical(const ADC_SampleData_t *raw, ADC_PhysicalData_t *physical);

#endif /* ADC_INTERFACE_H_ */
