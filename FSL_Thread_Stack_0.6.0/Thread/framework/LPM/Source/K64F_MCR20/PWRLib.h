/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PWRLib.h
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef __PWR_LIB_H__
#define __PWR_LIB_H__

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 * Note that it is not a good practice to include header files into header   *
 * files, so use this section only if there is no other better solution.     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

 #include "PWR_Interface.h"

/*****************************************************************************
 *                             PUBLIC MACROS                                *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/


/*****************************************************************************
 *                             PRIVATE MACROS                                *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...
 *---------------------------------------------------------------------------*
 *****************************************************************************/


#define cRAM2_POWERED_ON              SMC_VLLSCTRL_RAM2PO_MASK
#define cRAM2_POWERED_OFF             0

/* Definitions for LPTMR timer setup */

// LPTMR prescaler clocking options

#define cLPTMR_Source_Int_MCGIRCLK    0
#define cLPTMR_Source_Int_LPO_1KHz    1
#define cLPTMR_Source_Ext_ERCLK32K    2
#define cLPTMR_Source_Ext_OSCERCLK    3

// LPTMR period for LPO 1KHz source clock
#define cLPTMR_PRS_00001ms           0xFF
#define cLPTMR_PRS_00002ms           kLptmrPrescalerDivide2
#define cLPTMR_PRS_00004ms           kLptmrPrescalerDivide4GlichFiltch2
#define cLPTMR_PRS_00008ms           kLptmrPrescalerDivide8GlichFiltch4
#define cLPTMR_PRS_00016ms           kLptmrPrescalerDivide16GlichFiltch8
#define cLPTMR_PRS_00032ms           kLptmrPrescalerDivide32GlichFiltch16
#define cLPTMR_PRS_00064ms           kLptmrPrescalerDivide64GlichFiltch32
#define cLPTMR_PRS_00128ms           kLptmrPrescalerDivide128GlichFiltch64
#define cLPTMR_PRS_00256ms           kLptmrPrescalerDivide256GlichFiltch128
#define cLPTMR_PRS_00512ms           kLptmrPrescalerDivide512GlichFiltch256
#define cLPTMR_PRS_01024ms           kLptmrPrescalerDivide1024GlichFiltch512
#define cLPTMR_PRS_02048ms           kLptmrPrescalerDivide2048lichFiltch1024
#define cLPTMR_PRS_04096ms           kLptmrPrescalerDivide4096GlichFiltch2048
#define cLPTMR_PRS_08192ms           kLptmrPrescalerDivide8192GlichFiltch4096
#define cLPTMR_PRS_16384ms           kLptmrPrescalerDivide16384GlichFiltch8192
#define cLPTMR_PRS_32768ms           kLptmrPrescalerDivide32768GlichFiltch16384
#define cLPTMR_PRS_65536ms           kLptmrPrescalerDivide65535GlichFiltch32768

// LPTMR period for LPO 32.768KHz source clock
#define cLPTMR_PRS_125_div_by_4096ms  0xFFFFFFFF
#define cLPTMR_PRS_125_div_by_2048ms  kLptmrPrescalerDivide2
#define cLPTMR_PRS_125_div_by_1024ms  kLptmrPrescalerDivide4GlichFiltch2
#define cLPTMR_PRS_125_div_by_512ms   kLptmrPrescalerDivide8GlichFiltch4
#define cLPTMR_PRS_125_div_by_256ms   kLptmrPrescalerDivide16GlichFiltch8
#define cLPTMR_PRS_125_div_by_128ms   kLptmrPrescalerDivide32GlichFiltch16
#define cLPTMR_PRS_125_div_by_64ms    kLptmrPrescalerDivide64GlichFiltch32
#define cLPTMR_PRS_125_div_by_32ms    kLptmrPrescalerDivide128GlichFiltch64
#define cLPTMR_PRS_125_div_by_16ms    kLptmrPrescalerDivide256GlichFiltch128
#define cLPTMR_PRS_125_div_by_8ms     kLptmrPrescalerDivide512GlichFiltch256
#define cLPTMR_PRS_125_div_by_4ms     kLptmrPrescalerDivide1024GlichFiltch512
#define cLPTMR_PRS_125_div_by_2ms     kLptmrPrescalerDivide2048lichFiltch1024
#define cLPTMR_PRS_0125ms             kLptmrPrescalerDivide4096GlichFiltch2048
#define cLPTMR_PRS_0250ms             kLptmrPrescalerDivide8192GlichFiltch4096
#define cLPTMR_PRS_0500ms             kLptmrPrescalerDivide16384GlichFiltch8192
#define cLPTMR_PRS_1000ms             kLptmrPrescalerDivide32768GlichFiltch16384
#define cLPTMR_PRS_2000ms             kLptmrPrescalerDivide65535GlichFiltch32768





/*****************************************************************************
 *                        PRIVATE TYPE DEFINITIONS                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the data types definitions: stuctures, unions,    *
 * enumerations, typedefs ...                                                *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

#define  gPWRLib_LLWU_WakeupPin_PTE1_c    kLlwuWakeupPin0
#define  gPWRLib_LLWU_WakeupPin_PTE2_c    kLlwuWakeupPin1
#define  gPWRLib_LLWU_WakeupPin_PTE4_c    kLlwuWakeupPin2
#define  gPWRLib_LLWU_WakeupPin_PTA4_c    kLlwuWakeupPin3
#define  gPWRLib_LLWU_WakeupPin_PTA13_c   kLlwuWakeupPin4
#define  gPWRLib_LLWU_WakeupPin_PTB0_c    kLlwuWakeupPin5
#define  gPWRLib_LLWU_WakeupPin_PTC1_c    kLlwuWakeupPin6
#define  gPWRLib_LLWU_WakeupPin_PTC3_c    kLlwuWakeupPin7
#define  gPWRLib_LLWU_WakeupPin_PTC4_c    kLlwuWakeupPin8
#define  gPWRLib_LLWU_WakeupPin_PTC5_c    kLlwuWakeupPin9
#define  gPWRLib_LLWU_WakeupPin_PTC6_c    kLlwuWakeupPin10
#define  gPWRLib_LLWU_WakeupPin_PTC11_c   kLlwuWakeupPin11
#define  gPWRLib_LLWU_WakeupPin_PTD0_c    kLlwuWakeupPin12
#define  gPWRLib_LLWU_WakeupPin_PTD2_c    kLlwuWakeupPin13
#define  gPWRLib_LLWU_WakeupPin_PTD4_c    kLlwuWakeupPin14
#define  gPWRLib_LLWU_WakeupPin_PTD6_c    kLlwuWakeupPin15

#define  gPWRLib_LLWU_KeyboardMask_c     ( (1 << gPWRLib_LLWU_WakeupPin_PTC4_c) \
                                         | (1 << gPWRLib_LLWU_WakeupPin_PTC5_c) \
                                         | (1 << gPWRLib_LLWU_WakeupPin_PTC6_c) )


#define    gPWRLib_LLWU_WakeupModule_LPTMR_c      kLlwuWakeupModule0
#define    gPWRLib_LLWU_WakeupModule_CMP0_c       kLlwuWakeupModule1
#define    gPWRLib_LLWU_WakeupModule_CMP1_c       kLlwuWakeupModule2
#define    gPWRLib_LLWU_WakeupModule_RTC_Alarm_c  kLlwuWakeupModule5


/*****************************************************************************
 *                               PUBLIC VARIABLES(External)                  *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have global      *
 * (project) scope.                                                          *
 * These variables / constants can be accessed outside this module.          *
 * These variables / constants shall be preceded by the 'extern' keyword in  *
 * the interface header.                                                     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/* Zigbee stack status */
extern PWRLib_StackPS_t PWRLib_StackPS;

/* For LVD function */

#if (cPWR_LVD_Enable == 2)
extern PWRLib_LVD_VoltageLevel_t PWRLib_LVD_SavedLevel;
#endif  // #if (cPWR_LVD_Enable == 2)



/*****************************************************************************
 *                            PUBLIC FUNCTIONS                               *
 *---------------------------------------------------------------------------*
 * Add to this section all the global functions prototype preceded (as a     *
 * good practice) by the keyword 'extern'                                    *
 *---------------------------------------------------------------------------*
 *****************************************************************************/


/*---------------------------------------------------------------------------
 * Name: PWRLib_Radio_Enter_Doze
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_Doze
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PWRLib_Radio_Enter_AutoDoze
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_AutoDoze
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PWRLib_Radio_Enter_Idle
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_Idle
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PWRLib_Radio_Enter_Hibernate
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_Hibernate
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PWRLib_LLWU_UpdateWakeupReason
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_LLWU_UpdateWakeupReason(void);


/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_ClockStart
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_LPTMR_ClockStart
(
  uint8_t  ClkMode,
  uint32_t Ticks
);

/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_ClockCheck
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint32_t PWRLib_LPTMR_ClockCheck
(
  void
);


/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_ClockStop
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_LPTMR_ClockStop
(
  void
);


/*---------------------------------------------------------------------------
 * Name: PWRLib_LVD_CollectLevel
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_LVD_VoltageLevel_t PWRLib_LVD_CollectLevel
(
  void
);

/******************************************************************************
 * Name: PWRLib_GetMacStateReq
 * Description: Get status from MAC. Functions just as Asp_GetMacStateReq().
 *
 * Parameter(s): - none
 * Return: - gAspMacStateIdle_c     : MAC ready for Sleep or DeepSleep
 *           gAspMacStateBusy_c     : Don't sleep
 *           gAspMacStateNotEmpty_c : MAC allows Wait
 ******************************************************************************/
uint8_t PWRLib_GetMacStateReq
(
  void
);

#endif /* __PWR_LIB_H__ */
