/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// EPWM1 -> PFC_A Pinmux
//
//
// EPWM1_A - GPIO Settings
//
#define GPIO_PIN_EPWM1_A 0
#define PFC_A_EPWMA_GPIO 0
#define PFC_A_EPWMA_PIN_CONFIG GPIO_0_EPWM1_A
//
// EPWM1_B - GPIO Settings
//
#define GPIO_PIN_EPWM1_B 1
#define PFC_A_EPWMB_GPIO 1
#define PFC_A_EPWMB_PIN_CONFIG GPIO_1_EPWM1_B

//
// EPWM2 -> PFC_B Pinmux
//
//
// EPWM2_A - GPIO Settings
//
#define GPIO_PIN_EPWM2_A 2
#define PFC_B_EPWMA_GPIO 2
#define PFC_B_EPWMA_PIN_CONFIG GPIO_2_EPWM2_A
//
// EPWM2_B - GPIO Settings
//
#define GPIO_PIN_EPWM2_B 3
#define PFC_B_EPWMB_GPIO 3
#define PFC_B_EPWMB_PIN_CONFIG GPIO_3_EPWM2_B

//
// EPWM3 -> PFC_C Pinmux
//
//
// EPWM3_A - GPIO Settings
//
#define GPIO_PIN_EPWM3_A 4
#define PFC_C_EPWMA_GPIO 4
#define PFC_C_EPWMA_PIN_CONFIG GPIO_4_EPWM3_A
//
// EPWM3_B - GPIO Settings
//
#define GPIO_PIN_EPWM3_B 5
#define PFC_C_EPWMB_GPIO 5
#define PFC_C_EPWMB_PIN_CONFIG GPIO_5_EPWM3_B

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
#define myADCA_BASE ADCA_BASE
#define myADCA_RESULT_BASE ADCARESULT_BASE
#define myADCA_V_LL_AB ADC_SOC_NUMBER0
#define myADCA_FORCE_V_LL_AB ADC_FORCE_SOC0
#define myADCA_SAMPLE_WINDOW_V_LL_AB 200
#define myADCA_TRIGGER_SOURCE_V_LL_AB ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_V_LL_AB ADC_CH_ADCIN1
#define myADCA_V_LL_BC ADC_SOC_NUMBER1
#define myADCA_FORCE_V_LL_BC ADC_FORCE_SOC1
#define myADCA_SAMPLE_WINDOW_V_LL_BC 200
#define myADCA_TRIGGER_SOURCE_V_LL_BC ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_V_LL_BC ADC_CH_ADCIN2
#define myADCA_V_LL_CA ADC_SOC_NUMBER2
#define myADCA_FORCE_V_LL_CA ADC_FORCE_SOC2
#define myADCA_SAMPLE_WINDOW_V_LL_CA 200
#define myADCA_TRIGGER_SOURCE_V_LL_CA ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_V_LL_CA ADC_CH_ADCIN3
#define myADCA_V_Ref_1V5 ADC_SOC_NUMBER3
#define myADCA_FORCE_V_Ref_1V5 ADC_FORCE_SOC3
#define myADCA_SAMPLE_WINDOW_V_Ref_1V5 200
#define myADCA_TRIGGER_SOURCE_V_Ref_1V5 ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_V_Ref_1V5 ADC_CH_ADCIN0
#define myADCA_T_LLC_Pri_MOS ADC_SOC_NUMBER4
#define myADCA_FORCE_T_LLC_Pri_MOS ADC_FORCE_SOC4
#define myADCA_SAMPLE_WINDOW_T_LLC_Pri_MOS 200
#define myADCA_TRIGGER_SOURCE_T_LLC_Pri_MOS ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_T_LLC_Pri_MOS ADC_CH_ADCIN4
#define myADCA_T_LLC_Sec_MOS ADC_SOC_NUMBER5
#define myADCA_FORCE_T_LLC_Sec_MOS ADC_FORCE_SOC5
#define myADCA_SAMPLE_WINDOW_T_LLC_Sec_MOS 200
#define myADCA_TRIGGER_SOURCE_T_LLC_Sec_MOS ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_T_LLC_Sec_MOS ADC_CH_ADCIN5
#define myADCA_T_Inv_3Ph_MOS ADC_SOC_NUMBER6
#define myADCA_FORCE_T_Inv_3Ph_MOS ADC_FORCE_SOC6
#define myADCA_SAMPLE_WINDOW_T_Inv_3Ph_MOS 200
#define myADCA_TRIGGER_SOURCE_T_Inv_3Ph_MOS ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_T_Inv_3Ph_MOS ADC_CH_ADCIN6
#define myADCA_T_Ambient ADC_SOC_NUMBER7
#define myADCA_FORCE_T_Ambient ADC_FORCE_SOC7
#define myADCA_SAMPLE_WINDOW_T_Ambient 200
#define myADCA_TRIGGER_SOURCE_T_Ambient ADC_TRIGGER_EPWM1_SOCA
#define myADCA_CHANNEL_T_Ambient ADC_CH_ADCIN10
void myADCA_init();

#define myADCB_BASE ADCB_BASE
#define myADCB_RESULT_BASE ADCBRESULT_BASE
#define myADCB_I_DC800 ADC_SOC_NUMBER0
#define myADCB_FORCE_I_DC800 ADC_FORCE_SOC0
#define myADCB_SAMPLE_WINDOW_I_DC800 200
#define myADCB_TRIGGER_SOURCE_I_DC800 ADC_TRIGGER_EPWM1_SOCA
#define myADCB_CHANNEL_I_DC800 ADC_CH_ADCIN0
#define myADCB_V_DC800_CapU ADC_SOC_NUMBER1
#define myADCB_FORCE_V_DC800_CapU ADC_FORCE_SOC1
#define myADCB_SAMPLE_WINDOW_V_DC800_CapU 200
#define myADCB_TRIGGER_SOURCE_V_DC800_CapU ADC_TRIGGER_EPWM1_SOCA
#define myADCB_CHANNEL_V_DC800_CapU ADC_CH_ADCIN2
#define myADCB_V_DC800_CapL ADC_SOC_NUMBER2
#define myADCB_FORCE_V_DC800_CapL ADC_FORCE_SOC2
#define myADCB_SAMPLE_WINDOW_V_DC800_CapL 200
#define myADCB_TRIGGER_SOURCE_V_DC800_CapL ADC_TRIGGER_EPWM1_SOCA
#define myADCB_CHANNEL_V_DC800_CapL ADC_CH_ADCIN3
#define myADCB_V_DC125 ADC_SOC_NUMBER3
#define myADCB_FORCE_V_DC125 ADC_FORCE_SOC3
#define myADCB_SAMPLE_WINDOW_V_DC125 200
#define myADCB_TRIGGER_SOURCE_V_DC125 ADC_TRIGGER_EPWM1_SOCA
#define myADCB_CHANNEL_V_DC125 ADC_CH_ADCIN4
void myADCB_init();

#define myADCC_BASE ADCC_BASE
#define myADCC_RESULT_BASE ADCCRESULT_BASE
#define myADCC_I_Phase_A ADC_SOC_NUMBER0
#define myADCC_FORCE_I_Phase_A ADC_FORCE_SOC0
#define myADCC_SAMPLE_WINDOW_I_Phase_A 200
#define myADCC_TRIGGER_SOURCE_I_Phase_A ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_I_Phase_A ADC_CH_ADCIN0
#define myADCC_I_Phase_B ADC_SOC_NUMBER1
#define myADCC_FORCE_I_Phase_B ADC_FORCE_SOC1
#define myADCC_SAMPLE_WINDOW_I_Phase_B 200
#define myADCC_TRIGGER_SOURCE_I_Phase_B ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_I_Phase_B ADC_CH_ADCIN1
#define myADCC_I_Phase_C ADC_SOC_NUMBER2
#define myADCC_FORCE_I_Phase_C ADC_FORCE_SOC2
#define myADCC_SAMPLE_WINDOW_I_Phase_C 200
#define myADCC_TRIGGER_SOURCE_I_Phase_C ADC_TRIGGER_EPWM1_SOCA
#define myADCC_CHANNEL_I_Phase_C ADC_CH_ADCIN2
void myADCC_init();


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// CLA Configurations
//
//*****************************************************************************
#define myCLA0_BASE CLA1_BASE

//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them global
// and the main CPU can make use of them.
//
__attribute__((interrupt)) void Cla1Task1();
__attribute__((interrupt)) void Cla1Task2();
__attribute__((interrupt)) void Cla1Task3();
__attribute__((interrupt)) void Cla1Task4();
__attribute__((interrupt)) void Cla1Task5();
__attribute__((interrupt)) void Cla1Task6();
__attribute__((interrupt)) void Cla1Task7();
__attribute__((interrupt)) void Cla1Task8();
void myCLA0_init();


//*****************************************************************************
//
// EPWM Configurations
//
//*****************************************************************************
#define PFC_A_BASE EPWM1_BASE
#define PFC_A_TBPRD 3125
#define PFC_A_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define PFC_A_TBPHS 0
#define PFC_A_CMPA 1470
#define PFC_A_CMPB 0
#define PFC_A_CMPC 0
#define PFC_A_CMPD 0
#define PFC_A_DBRED 150
#define PFC_A_DBFED 150
#define PFC_A_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PFC_A_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PFC_A_INTERRUPT_SOURCE EPWM_INT_TBCTR_ZERO
#define PFC_B_BASE EPWM2_BASE
#define PFC_B_TBPRD 3125
#define PFC_B_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define PFC_B_TBPHS 0
#define PFC_B_CMPA 1470
#define PFC_B_CMPB 0
#define PFC_B_CMPC 0
#define PFC_B_CMPD 0
#define PFC_B_DBRED 150
#define PFC_B_DBFED 150
#define PFC_B_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PFC_B_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PFC_B_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED
#define PFC_C_BASE EPWM3_BASE
#define PFC_C_TBPRD 3125
#define PFC_C_COUNTER_MODE EPWM_COUNTER_MODE_UP_DOWN
#define PFC_C_TBPHS 0
#define PFC_C_CMPA 1470
#define PFC_C_CMPB 0
#define PFC_C_CMPC 0
#define PFC_C_CMPD 0
#define PFC_C_DBRED 150
#define PFC_C_DBFED 150
#define PFC_C_TZA_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PFC_C_TZB_ACTION EPWM_TZ_ACTION_HIGH_Z
#define PFC_C_INTERRUPT_SOURCE EPWM_INT_TBCTR_DISABLED

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_myADCA_1
// ISR need to be defined for the registered interrupts
#define INT_myADCA_1 INT_ADCA1
#define INT_myADCA_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void ADC_FrameDone_INT(void);

//*****************************************************************************
//
// MEMCFG Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// SYNC Scheme Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	ADC_init();
void	ASYSCTL_init();
void	CLA_init();
void	EPWM_init();
void	INTERRUPT_init();
void	MEMCFG_init();
void	SYNC_init();
void	SYSCTL_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
