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
#define cLPTMR_PRS_00001ms           LPTMR_PDD_DIVIDER_1     
#define cLPTMR_PRS_00002ms           LPTMR_PDD_DIVIDER_2     
#define cLPTMR_PRS_00004ms           LPTMR_PDD_DIVIDER_4     
#define cLPTMR_PRS_00008ms           LPTMR_PDD_DIVIDER_8     
#define cLPTMR_PRS_00016ms           LPTMR_PDD_DIVIDER_16    
#define cLPTMR_PRS_00032ms           LPTMR_PDD_DIVIDER_32    
#define cLPTMR_PRS_00064ms           LPTMR_PDD_DIVIDER_64    
#define cLPTMR_PRS_00128ms           LPTMR_PDD_DIVIDER_128   
#define cLPTMR_PRS_00256ms           LPTMR_PDD_DIVIDER_256   
#define cLPTMR_PRS_00512ms           LPTMR_PDD_DIVIDER_512   
#define cLPTMR_PRS_01024ms           LPTMR_PDD_DIVIDER_1024  
#define cLPTMR_PRS_02048ms           LPTMR_PDD_DIVIDER_2048  
#define cLPTMR_PRS_04096ms           LPTMR_PDD_DIVIDER_4096  
#define cLPTMR_PRS_08192ms           LPTMR_PDD_DIVIDER_8192  
#define cLPTMR_PRS_16384ms           LPTMR_PDD_DIVIDER_16384 
#define cLPTMR_PRS_32768ms           LPTMR_PDD_DIVIDER_32768 
#define cLPTMR_PRS_65536ms           LPTMR_PDD_DIVIDER_65536 

// LPTMR period for LPO 32.768KHz source clock
#define cLPTMR_PRS_125_div_by_4096ms  LPTMR_PDD_DIVIDER_1 
#define cLPTMR_PRS_125_div_by_2048ms  LPTMR_PDD_DIVIDER_2 
#define cLPTMR_PRS_125_div_by_1024ms  LPTMR_PDD_DIVIDER_4 
#define cLPTMR_PRS_125_div_by_512ms   LPTMR_PDD_DIVIDER_8 
#define cLPTMR_PRS_125_div_by_256ms   LPTMR_PDD_DIVIDER_16 
#define cLPTMR_PRS_125_div_by_128ms   LPTMR_PDD_DIVIDER_32 
#define cLPTMR_PRS_125_div_by_64ms    LPTMR_PDD_DIVIDER_64 
#define cLPTMR_PRS_125_div_by_32ms    LPTMR_PDD_DIVIDER_128 
#define cLPTMR_PRS_125_div_by_16ms    LPTMR_PDD_DIVIDER_256 
#define cLPTMR_PRS_125_div_by_8ms     LPTMR_PDD_DIVIDER_512 
#define cLPTMR_PRS_125_div_by_4ms     LPTMR_PDD_DIVIDER_1024 
#define cLPTMR_PRS_125_div_by_2ms     LPTMR_PDD_DIVIDER_2048 
#define cLPTMR_PRS_0125ms             LPTMR_PDD_DIVIDER_4096 
#define cLPTMR_PRS_0250ms             LPTMR_PDD_DIVIDER_8192 
#define cLPTMR_PRS_0500ms             LPTMR_PDD_DIVIDER_16384 
#define cLPTMR_PRS_1000ms             LPTMR_PDD_DIVIDER_32768 
#define cLPTMR_PRS_2000ms             LPTMR_PDD_DIVIDER_65536 

#define gLPOCalibrationOffest_c 0
#define gLPOCalibrationTicks_c 100



/*****************************************************************************
 *                        PRIVATE TYPE DEFINITIONS                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the data types definitions: stuctures, unions,    *
 * enumerations, typedefs ...                                                *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

#define  gPWRLib_LLWU_WakeupPin_PTB0_c    LLWU_EXT_PIN5
#define  gPWRLib_LLWU_WakeupPin_PTC1_c    LLWU_EXT_PIN6   
#define  gPWRLib_LLWU_WakeupPin_PTC3_c    LLWU_EXT_PIN7
#define  gPWRLib_LLWU_WakeupPin_PTC4_c    LLWU_EXT_PIN8
#define  gPWRLib_LLWU_WakeupPin_PTC5_c    LLWU_EXT_PIN9
#define  gPWRLib_LLWU_WakeupPin_PTC6_c    LLWU_EXT_PIN10
#define  gPWRLib_LLWU_WakeupPin_PTC11_c   LLWU_EXT_PIN11
#define  gPWRLib_LLWU_WakeupPin_PTD0_c    LLWU_EXT_PIN12
#define  gPWRLib_LLWU_WakeupPin_PTD2_c    LLWU_EXT_PIN13
#define  gPWRLib_LLWU_WakeupPin_PTD4_c    LLWU_EXT_PIN14
#define  gPWRLib_LLWU_WakeupPin_PTD6_c    LLWU_EXT_PIN15 

#define  gPWRLib_LLWU_KeyboardMask_c (LLWU_EXT_PIN15)

#define    gPWRLib_LLWU_WakeupModule_LPTMR_c      LLWU_INT_MODULE0
#define    gPWRLib_LLWU_WakeupModule_CMP0_c       LLWU_INT_MODULE1
#define    gPWRLib_LLWU_WakeupModule_RTC_Alarm_c  LLWU_INT_MODULE5


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
 * Name: PWRLib_MCUEnter_VLPS
 * Description: Puts the processor into VLPS (Very Low Power Stop).

                Mode of operation details:
                 - ARM core enters DeepSleep Mode
                 - ARM core is clock gated (HCLK = OFF)
                 - NVIC is disable (FCLK = OFF)
                 - WIC is used to wake up from interruptions
                 - Platform and peripheral clock are stopped
                 - MCG module can be configured to leave reference clocks running
                 - On chip voltage regulator is in a mode that supplies only enough
                   power to run the MCU in a reduced frequency
                 - All SRAM is operating (content retained and I/O states held)

                VLPS mode is exited into RUN mode using any enabled interrupt (with LPWUI =1) or RESET.
                
                The AVLP must be set to 0b1 in MC_PMPROT register in order to allow VPLS mode.

 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_MCU_Enter_VLPS
(
  void
);

/*---------------------------------------------------------------------------
 * Name: PWRLib_MCUEnter_VLLS1
 * Description: Puts the processor into VLLS1 (Very Low Leakage Stop2).

                Mode of operation details:
                 - ARM core enters SleepDeep Mode
                 - ARM core is clock gated (HCLK = OFF)
                 - NVIC is disable (FCLK = OFF)
                 - LLWU should configure by user to enable the desire wake up source
                 - Platform and peripheral clock are stopped
                 - MCG module can be configured to leave reference clocks running
                 - On chip voltage regulator is in a mode that supplies only enough
                   power to run the MCU in a reduced frequency
                 - SRAM_L and SRAM_H is powered off.
                 - Most modules are disabled

                VLLS1 mode is exited into RUN mode using LLWU module or RESET.
                All wakeup goes through Reset sequence.

                The AVLLS1 must be set to 0b1 in MC_PMPROT register in order to allow VLLS2 mode.

 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_MCU_Enter_VLLS1
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
 * Name: PWRLib_LPTMR_PhyTicksToLPOTicks
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint32_t  PWRLib_LPTMR_PhyTicksToLPOTicks
(
 phyTime_t phyTicks
);
/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_LPOTicksToPhyTicks
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
phyTime_t  PWRLib_LPTMR_LPOTicksToPhyTicks
(
 uint32_t lpoTicks
);

/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_GetCounterValue
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint32_t  PWRLib_LPTMR_GetCounterValue
(
void
);
/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_IsWakeUpTimeExpired
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t  PWRLib_LPTMR_IsWakeUpTimeExpired
(
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
