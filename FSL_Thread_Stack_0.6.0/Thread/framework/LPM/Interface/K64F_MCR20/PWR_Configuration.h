/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PWR_Configuraion.h
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


#ifndef _PWR_CONFIGURATION_H_
#define _PWR_CONFIGURATION_H_

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 * Note that it is not a good practice to include header files into header   *
 * files, so use this section only if there is no other better solution.     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#include "Keyboard.h"
#include "TimersManager.h"

#include "LPMConfig.h"

/************************************************************************************
*************************************************************************************
* Module configuration constants
*************************************************************************************
************************************************************************************/

/*********************/
/* LVD configuration */
/*********************/

//-----------------------------------------------------------------------------
// The use of Low Voltage detection has the following possibilities:
//   0: Don't use Low voltage detection at all
//   1: Use polled => Check made each time the function is called.
//   2: A minutes software timer used for handling when to poll LVD, according
//      to the cPWR_LVD_Ticks constant
//   3: LVDRE  are set to hold MCU in reset while VLVDL  condition is detected
// PEX Settings: - in modes 0,1,2 the property CPU->Internal peripherals->Power management controller->LVDreset must be Disabled
//               - in mode 3 the property CPU->Internal peripherals->Power management controller->LVDreset must be Enabled  
//The the propery refers to LVDRE bit in the PMC_LVDSC1 register which is a write once bit, so it cannot be modified after that.

#ifndef cPWR_LVD_Enable
  #define cPWR_LVD_Enable                         0
#endif

//-----------------------------------------------------------------------------
// How often to check the LVD level when cPWR_LVD_Enable == 2
// This is the number of minutes before voltage is checked (Consumes
// current and time)

#ifndef cPWR_LVD_Ticks
  #define cPWR_LVD_Ticks                          60
#endif

//-----------------------------------------------------------------------------
// To enable/disable all of the code in this PWR/PWRLib files.
//   TRUE =  1: Use PowerDown functions (Normal)
//   FALSE = 0: Don't use PowerDown. Will cut variables and code out. But
//              functions still exist. Useful for debugging and test purposes
#ifndef cPWR_UsePowerDownMode
  #define cPWR_UsePowerDownMode                   1
#endif

#ifndef cPWR_UsePowerModuleStandAlone
  #define cPWR_UsePowerModuleStandAlone           0
#endif

//-----------------------------------------------------------------------------
// The following define configures whether to test or not if the RTC oscillator has started on entering low power,
// for the low power modes that use it. 
//   TRUE =  1: if the RTC oscillator didn't started, the system enters sleep instead of deep sleep 

//   FALSE = 0: no RTC tests are made and the system enters deep sleep

#ifndef gPWR_EnsureOscStabilized_d
#define gPWR_EnsureOscStabilized_d                0
#endif

//-----------------------------------------------------------------------------
// The way that DeepSleep mode are handled. Following possibilities exist:
//*****************************************************************************
//   0: No DeepSleep done, but application can set modes
//*****************************************************************************
//   1: MCU/Radio low power modes:
//      MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//      Radio in hibernate mode.  
//      Wakeup sources:
//      On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. Wakeup goes through Reset sequence.
//*****************************************************************************
//   2: MCU/Radio low power modes:
//        MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//        Radio in hibernate mode.  
//      Wakeup sources:
//        LPTMR interrupt using LLWU module. Wakeup goes through Reset sequence.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   LPO(internal low power oscillator)
//          - LPTMR resolution:     possible values: 1 ms, 2 ms, 4 ms, 8 ms, 16 ms, 32 ms, 64 ms, 128 ms, 256 ms,
//                                  512 ms, 1024 ms, 2048 ms, 4096 ms, 8192 ms, 16384 ms, 32768 ms, 65536 ms.
//                                  See PWRLib.h for details.
//          - Deep sleep timeout:   cPWR_TMRTicks * cPWR_LPTMRTickTime
//*****************************************************************************
//   3: MCU/Radio low power modes:
//        MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//        Radio in hibernate mode.  
//      Wakeup sources:
//        LPTMR interrupt using LLWU module. Wakeup goes through Reset sequence.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   ERCLK32K(secondary external reference clock. 32.768 kHz crystal connected to RTC oscillator)
//          - LPTMR resolution:     possible values: 125/4096 ms, 125/2048 ms, 125/1024 ms, 125/512 ms, 125/256 ms, 125/128 ms,
//                                  125/64 ms, 125/32 ms, 125/16 ms, 125/8 ms, 125/4 ms, 125/2 ms, 125 ms,
//                                  250 ms, 500 ms, 1000 ms, 2000 ms
//                                  See PWRLib.h for details.
//          - Deep sleep timeout:   cPWR_TMRTicks * cPWR_LPTMRTickTime
//*****************************************************************************
//   4: MCU/Radio low power modes:
//        MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//        Radio in hibernate mode.  
//      Wakeup sources:
//        RTC interrupt using LLWU module. Wakeup goes through Reset sequence.
//          - RTC wakeup timeout:   fixed at compile time.
//          - RTC clock source:     32.768 kHz crystal connected to RTC oscillator
//          - RTC resolution:       1 s.
//          - Deep sleep timeout:   cPWR_TMRTicks * 1s
//*****************************************************************************
//   5: MCU/Radio low power modes:
//        MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. Wakeup goes through Reset sequence.
//        LPTMR interrupt using LLWU module. Wakeup goes through Reset sequence.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   LPO(internal low power oscillator)
//          - LPTMR resolution:     possible values: 1 ms, 2 ms, 4 ms, 8 ms, 16 ms, 32 ms, 64 ms, 128 ms, 256 ms,
//                                  512 ms, 1024 ms, 2048 ms, 4096 ms, 8192 ms, 16384 ms, 32768 ms, 65536 ms.
//                                  See PWRLib.h for details.
//          - Deep sleep timeout:   cPWR_TMRTicks * cPWR_LPTMRTickTime
//*****************************************************************************
//   6: MCU/Radio low power modes:
//        MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. Wakeup goes through Reset sequence.
//        LPTMR interrupt using LLWU module. Wakeup goes through Reset sequence.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   ERCLK32K(secondary external reference clock. 32.768 kHz crystal connected to RTC oscillator)
//          - LPTMR resolution:     possible values: 125/4096 ms, 125/2048 ms, 125/1024 ms, 125/512 ms, 125/256 ms, 125/128 ms,
//                                  125/64 ms, 125/32 ms, 125/16 ms, 125/8 ms, 125/4 ms, 125/2 ms, 125 ms,
//                                  250 ms, 500 ms, 1000 ms, 2000 ms
//                                  See PWRLib.h for details.
//          - Deep sleep timeout:   cPWR_TMRTicks * cPWR_LPTMRTickTime
//*****************************************************************************
//   7: MCU/Radio low power modes:
//        MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. Wakeup goes through Reset sequence.
//        RTC interrupt using LLWU module. Wakeup goes through Reset sequence.
//          - RTC wakeup timeout:   fixed at compile time.
//          - RTC clock source:     32.768 kHz crystal connected to RTC oscillator
//          - RTC resolution:       1 s.
//          - Deep sleep timeout:   cPWR_TMRTicks * 1s
//*****************************************************************************
//   8: MCU/Radio low power modes:
//        MCU in VLLS2 mode (only portion of SRAM_U remains powered on).
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. Wakeup goes through Reset sequence.
//        LPTMR interrupt using LLWU module. Wakeup goes through Reset sequence.
//          - LPTMR wakeup timeout: configurable at run time.
//          - LPTMR clock source:   LPO(internal low power oscillator)
//          - LPTMR resolution:     fixed to 2 ms (125 802.15.4 PHY symbols)
//          - Deep sleep timeout:   cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs or 
//                                  PWR_SetDeepSleepTimeInSymbols to change it at run time. 
//*****************************************************************************
//   9: MCU/Radio low power modes:
//        MCU in LLS mode.
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. 
//        LPTMR interrupt using LLWU module.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   LPO(internal low power oscillator)
//          - LPTMR resolution:     possible values: 1 ms, 2 ms, 4 ms, 8 ms, 16 ms, 32 ms, 64 ms, 128 ms, 256 ms,
//                                  512 ms, 1024 ms, 2048 ms, 4096 ms, 8192 ms, 16384 ms, 32768 ms, 65536 ms.
//                                  See PWRLib.h for details.
//          - Deep sleep timeout:   cPWR_TMRTicks * cPWR_LPTMRTickTime
//*****************************************************************************
//   10: MCU/Radio low power modes:
//        MCU in LLS mode.
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. 
//        LPTMR interrupt using LLWU module.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   ERCLK32K(secondary external reference clock. 32.768 kHz crystal connected to RTC oscillator)
//          - LPTMR resolution:     possible values: 125/4096 ms, 125/2048 ms, 125/1024 ms, 125/512 ms, 125/256 ms, 125/128 ms,
//                                  125/64 ms, 125/32 ms, 125/16 ms, 125/8 ms, 125/4 ms, 125/2 ms, 125 ms,
//                                  250 ms, 500 ms, 1000 ms, 2000 ms
//                                  See PWRLib.h for details.
//          - Deep sleep timeout:   cPWR_TMRTicks * cPWR_LPTMRTickTime
//*****************************************************************************
//   11: MCU/Radio low power modes:
//        MCU in LLS mode.
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. 
//        RTC interrupt using LLWU module.
//          - RTC wakeup timeout:   fixed at compile time.
//          - RTC clock source:     32.768 kHz crystal connected to RTC oscillator
//          - RTC resolution:       1 s.
//          - Deep sleep timeout:   cPWR_TMRTicks * 1s 
//*****************************************************************************
//   12: MCU/Radio low power modes:
//        MCU in LLS mode.
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt using LLWU module. 
//        LPTMR interrupt using LLWU module.
//          - LPTMR wakeup timeout: configurable at run time.
//          - LPTMR clock source:   LPO(internal low power oscillator)
//          - LPTMR resolution:     fixed to 2 ms (125 802.15.4 PHY symbols)
//          - Deep sleep timeout:   cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs or 
//                                  PWR_SetDeepSleepTimeInSymbols to change it at run time.
//*****************************************************************************
//   13: MCU/Radio low power modes:
//        MCU in VLPS mode.
//        Radio in DOZE mode.
//      Wakeup sources:
//        On TWR_KW21D512:           GPIO (push button) interrupt.
//        UART interrupt.
//        Radio timer interrupt.
//          - Radio timer wakeup timeout: configurable at run time.
//          - Radio timer resolution:     fixed to 16 us (one 802.15.4 PHY symbol)
//          - Deep sleep timeout:   cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs or 
//                                  PWR_SetDeepSleepTimeInSymbols to change it at run time.
//*****************************************************************************
//   14: MCU/Radio low power modes:
//        MCU in LLS mode.
//        Radio in hibernate mode.  
//      Wakeup sources:
//        On TWR_KW21D512:           - GPIO (push button) interrupt using LLWU module. 
//                                   - LPTMR interrupt using LLWU module.
//                                     Wakeup goes through Reset sequence.
//          - LPTMR wakeup timeout: the shortest timeout period of any active LPTMR-based timer (see Timer.c) 
//          - LPTMR resolution:     1 ms
//*****************************************************************************
//   15: MCU/Radio low power modes:
//        MCU in LLS mode.
//        Radio power mode is managed by MAC
//      Wakeup sources:
//        On TWR_KW21D512:           - GPIO (push button) interrupt using LLWU module. 
//                                   - LPTMR interrupt using LLWU module.
//                                     Wakeup goes through Reset sequence.
//          - LPTMR wakeup timeout: the shortest timeout period of any active LPTMR-based timer (see Timer.c) 
//          - LPTMR resolution:     1 ms
//*****************************************************************************

#ifndef cPWR_DeepSleepMode
  #define cPWR_DeepSleepMode                     10
#endif

//-----------------------------------------------------------------------------
// The way that Sleep mode are handled. Following possibilities exist:
//   0: No Sleep done, but application can set modes
//   1: MCU/Radio low power modes:
//        MCU in WAIT mode.
//        Radio in normal mode.

#ifndef cPWR_SleepMode
#define cPWR_SleepMode                            1
#endif

//-----------------------------------------------------------------------------
// Whether to run LPTMR from internal LPO or external oscillator
#define LPTMR_SOURCE_LPO1KHZ  (0x01)
#define LPTMR_SOURCE_EXT32KHZ (0x02)

#ifndef cPWR_LPTMRClockSource
 #if ( (cPWR_DeepSleepMode == 2) || (cPWR_DeepSleepMode == 5) || (cPWR_DeepSleepMode == 8) || (cPWR_DeepSleepMode == 9)  || (cPWR_DeepSleepMode == 12) )
   #define cPWR_LPTMRClockSource                   LPTMR_SOURCE_LPO1KHZ
 #else 
   #define cPWR_LPTMRClockSource                   LPTMR_SOURCE_EXT32KHZ
 #endif
#endif

#ifndef cPWR_LPTMRTickTimeSource_LPO_1KHz
 #define cPWR_LPTMRTickTimeSource_LPO_1KHz       cLPTMR_PRS_01024ms
#endif

#ifndef cPWR_LPTMRTickTimeSource_ERCLK32K
 #define cPWR_LPTMRTickTimeSource_ERCLK32K       cLPTMR_PRS_1000ms
#endif
//-----------------------------------------------
// This define represents the LPTMR tick time (resolution) and depends on the cPWR_LPTMRClockSource
/* BEGIN Set cPWR_LPTMRTickTime */

#ifndef cPWR_LPTMRTickTime

#if (cPWR_LPTMRClockSource == LPTMR_PDD_SOURCE_LPO1KHZ)
  #define cPWR_LPTMRTickTime                      cPWR_LPTMRTickTimeSource_LPO_1KHz
#else
  #define cPWR_LPTMRTickTime                      cPWR_LPTMRTickTimeSource_ERCLK32K
#endif //(cPWR_LPTMRClockSource == LPTMR_PDD_SOURCE_LPO1KHZ)

#endif //cPWR_LPTMRTickTime
/* END Set cPWR_LPTMRTickTime */
//-----------------------------------------------------------------------------
// This number time gives the time to DeepSleep:
//  - when the LPTMR (see cPWR_DeepSleepMode) is used the time to deep sleep is
//    cPWR_TMRTicks * cPWR_LPTMRTickTime
//  - when the RTC (see cPWR_DeepSleepMode) is used the time to deep speep is
//    cPWR_TMRTicks * 1s
//
#ifndef cPWR_TMRTicks
 #define cPWR_TMRTicks                               3
#endif

//-----------------------------------------------------------------------------
// The deep sleep duration in ms. Default value of 3072 ms set to match the
// default timeout set when using LPTMR as wakeup source.
#ifndef cPWR_DeepSleepDurationMs
  #define cPWR_DeepSleepDurationMs                3000
#endif

//-----------------------------------------------------------------------------
// Enabling of external call to a procedure each time that DeepSleep are exited
//   0: Don't call any functions after DeepSleep (MAC)
//   1: Call a function after DeepSleep (Stack)
#ifndef cPWR_CallWakeupStackProcAfterDeepSleep
  #define cPWR_CallWakeupStackProcAfterDeepSleep  0
#endif

//-----------------------------------------------------------------------------
// The extra function to call every time RTI clock run's out. Used by Stack.
#if (cPWR_CallWakeupStackProcAfterDeepSleep == 0)
  #define cPWR_DeepSleepWakeupStackProc           ;
#else
  extern void                                     DeepSleepWakeupStackProc(void);
  #define cPWR_DeepSleepWakeupStackProc           DeepSleepWakeupStackProc();  
#endif

//-----------------------------------------------------------------------------

#if (cPWR_LVD_Enable > 3)
  #error "*** ERROR: Illegal value in cPWR_LVD_Enable"
#endif

#if (cPWR_LVD_Enable == 2)
  #if (!gTMR_Enabled_d) 
    #error "*** ERROR: Illegal value in cPWR_LVD_Enable"
  #endif
#endif

#if (cPWR_UsePowerDownMode > 1)
  #error "*** ERROR: Illegal value in cPWR_UsePowerDownMode"
#endif

#if (cPWR_CallWakeupStackProcAfterDeepSleep > 1)
  #error "*** ERROR: Illegal value in cPWR_CallWakeupStackProcAfterDeepSleep"
#endif

#if (cPWR_DeepSleepMode > 15)
  #error "*** ERROR: Illegal value in cPWR_DeepSleepMode"
#endif

#if (cPWR_SleepMode > 1)
  #error "*** ERROR: Illegal value in cPWR_SleepMode"
#endif

#if (gTMR_EnableHWLowPowerTimers_d)
  #if (!cPWR_UsePowerDownMode)
    #error "Hardware low power timers can only be used when PWR module is enabled"
  #elif (cPWR_DeepSleepMode != 14) && (cPWR_DeepSleepMode != 15)
    #error "Hardware low power timers can only be used in deep sleep mode 14 and 15"
  #endif
#endif

#endif /* _PWR_CONFIGURATION_H_ */
