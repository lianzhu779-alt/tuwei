#include "adc_interface.h"
#include "cla_shared.h"
#include <math.h>

//
// 全局ADC采样数据结构
//
ADC_SampleData_t g_ADC_Data = {0};
ADC_PhysicalData_t g_ADC_PhysicalData = {0};

#pragma DATA_SECTION(g_CLA_CpuToClaAdcData, "CpuToCla1MsgRAM");
#pragma DATA_ALIGN(g_CLA_CpuToClaAdcData, 2)
volatile CLA_CpuToClaAdcData_t g_CLA_CpuToClaAdcData = {0};

static void ADC_UpdateClaMessage(void);

//
// 读取所有ADC采样结果
//
void ADC_ReadAllResults(ADC_SampleData_t *data)
{
    //
    // 读取ADCA结果
    //
    data->V_LL_AB       = ADC_readResult(myADCA_RESULT_BASE, myADCA_V_LL_AB);
    data->V_LL_BC       = ADC_readResult(myADCA_RESULT_BASE, myADCA_V_LL_BC);
    data->V_LL_CA       = ADC_readResult(myADCA_RESULT_BASE, myADCA_V_LL_CA);
    data->V_Ref_1V5     = ADC_readResult(myADCA_RESULT_BASE, myADCA_V_Ref_1V5);
    data->T_LLC_Pri_MOS = ADC_readResult(myADCA_RESULT_BASE, myADCA_T_LLC_Pri_MOS);
    data->T_LLC_Sec_MOS = ADC_readResult(myADCA_RESULT_BASE, myADCA_T_LLC_Sec_MOS);
    data->T_Inv_3Ph_MOS = ADC_readResult(myADCA_RESULT_BASE, myADCA_T_Inv_3Ph_MOS);
    data->T_Ambient     = ADC_readResult(myADCA_RESULT_BASE, myADCA_T_Ambient);

    //
    // 读取ADCB结果
    //
    data->I_DC800      = ADC_readResult(myADCB_RESULT_BASE, myADCB_I_DC800);
    data->V_DC800_CapU = ADC_readResult(myADCB_RESULT_BASE, myADCB_V_DC800_CapU);
    data->V_DC800_CapL = ADC_readResult(myADCB_RESULT_BASE, myADCB_V_DC800_CapL);
    data->V_DC125      = ADC_readResult(myADCB_RESULT_BASE, myADCB_V_DC125);

    //
    // 读取ADCC结果
    //
    data->I_Phase_A = ADC_readResult(myADCC_RESULT_BASE, myADCC_I_Phase_A);
    data->I_Phase_B = ADC_readResult(myADCC_RESULT_BASE, myADCC_I_Phase_B);
    data->I_Phase_C = ADC_readResult(myADCC_RESULT_BASE, myADCC_I_Phase_C);
}



//
// 将原始ADC数据转换为物理值
//
void ADC_ConvertToPhysical(const ADC_SampleData_t *raw, ADC_PhysicalData_t *physical)
{
    //
    // 转换线电压 - 使用Vref作为零点参考
    // 注意: 必须先转换为有符号类型,避免uint16_t减法下溢
    //
    physical->V_LL_AB = (float)((int16_t)raw->V_LL_AB - (int16_t)raw->V_Ref_1V5) * V_LL_SCALE;
    physical->V_LL_BC = (float)((int16_t)raw->V_LL_BC - (int16_t)raw->V_Ref_1V5) * V_LL_SCALE;
    physical->V_LL_CA = (float)((int16_t)raw->V_LL_CA - (int16_t)raw->V_Ref_1V5) * V_LL_SCALE;
    physical->V_Ref_1V5 = (float)((uint16_t)raw->V_Ref_1V5) * ADC_TO_VOLT;

    //
    // 转换直流母线测量值
    // 显式类型转换确保数值精度和类型安全
    //
    physical->I_DC800 = (float)((uint16_t)raw->I_DC800) * I_DC800_K1 - I_DC800_K2;
    physical->V_DC800_CapU = (float)((uint16_t)raw->V_DC800_CapU) * V_DC800_CAP_SCALE;
    physical->V_DC800_CapL = (float)((uint16_t)raw->V_DC800_CapL) * V_DC800_CAP_SCALE;
    physical->V_DC800_Total = physical->V_DC800_CapU + physical->V_DC800_CapL;
    physical->V_DC125 = (float)((uint16_t)raw->V_DC125) * V_DC125_SCALE;

    //
    // 转换三相电流 - 使用预计算常量避免除法和重复乘法
    // 每相使用不同的偏移量补偿值,显式类型转换确保精度
    //
    physical->I_Phase_A = (float)((uint16_t)raw->I_Phase_A) * I_PHASE_K1 - I_PHASE_A_K2;
    physical->I_Phase_B = (float)((uint16_t)raw->I_Phase_B) * I_PHASE_K1 - I_PHASE_B_K2;
    physical->I_Phase_C = (float)((uint16_t)raw->I_Phase_C) * I_PHASE_K1 - I_PHASE_C_K2;

    //
    // 转换温度传感器读数 - 暂时转换为电压值
    // TODO: 根据具体温度传感器型号添加温度转换公式
    //
    physical->T_LLC_Pri_MOS = (float)((uint16_t)raw->T_LLC_Pri_MOS) * ADC_TO_VOLT;
    physical->T_LLC_Sec_MOS = (float)((uint16_t)raw->T_LLC_Sec_MOS) * ADC_TO_VOLT;
    physical->T_Inv_3Ph_MOS = (float)((uint16_t)raw->T_Inv_3Ph_MOS) * ADC_TO_VOLT;
    physical->T_Ambient = (float)((uint16_t)raw->T_Ambient) * ADC_TO_VOLT;
}

//
// 将实时物理量拷贝到消息RAM，供CLA读取
//
static void ADC_UpdateClaMessage(void)
{
    g_CLA_CpuToClaAdcData.V_LL_AB = g_ADC_PhysicalData.V_LL_AB;
    g_CLA_CpuToClaAdcData.V_LL_BC = g_ADC_PhysicalData.V_LL_BC;
    g_CLA_CpuToClaAdcData.V_LL_CA = g_ADC_PhysicalData.V_LL_CA;
    g_CLA_CpuToClaAdcData.V_DC800_Total = g_ADC_PhysicalData.V_DC800_Total;
    g_CLA_CpuToClaAdcData.I_Phase_A = g_ADC_PhysicalData.I_Phase_A;
    g_CLA_CpuToClaAdcData.I_Phase_B = g_ADC_PhysicalData.I_Phase_B;
    g_CLA_CpuToClaAdcData.I_Phase_C = g_ADC_PhysicalData.I_Phase_C;
}

//
// ADC中断服务程序 - ADCA INT1
// 当所有ADCA的SOC转换完成时触发（基于SOC7）
//
__interrupt void ADC_FrameDone_INT(void)
{
    //
    // 读取所有ADC结果到全局数据结构
    //
    ADC_ReadAllResults(&g_ADC_Data);

    //
    // 转换为物理值
    //
    ADC_ConvertToPhysical(&g_ADC_Data, &g_ADC_PhysicalData);
    ADC_UpdateClaMessage();

    //
    // 清除ADC中断标志
    //
    ADC_clearInterruptStatus(myADCA_BASE, ADC_INT_NUMBER1);

    //
    // 应答PIE中断组
    //
    Interrupt_clearACKGroup(INT_myADCA_1_INTERRUPT_ACK_GROUP);
}
