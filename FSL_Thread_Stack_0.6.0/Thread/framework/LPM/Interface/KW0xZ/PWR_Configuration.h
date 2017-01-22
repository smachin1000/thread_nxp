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
  #define cPWR_UsePowerDownMode                   TRUE
#endif

#ifndef cPWR_UsePowerModuleStandAlone
  #define cPWR_UsePowerModuleStandAlone           0
#endif



//-----------------------------------------------------------------------------
// The way that DeepSleep mode are handled. Following possibilities exist:
//*****************************************************************************
//   0: No DeepSleep done, but application can set modes
//*****************************************************************************
//   1: MCU/Radio low power modes:
//        MCU in LLS mode.
//        Radio in sleep mode.  
//      Wakeup sources:
//        GPIO (push button) interrupt using LLWU module(SW3). 
//        LPTMR interrupt using LLWU module.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   LPO(internal low power oscillator) calibrated versus Phy Timer period
//          - LPTMR resolution:     LPO period(prescaler = 1)
//                                  
//          - Deep sleep timeout:   cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs or 
//                                  to change it at run time. MAC can set an absolute time at which the system must be up.
//      PEx settings: CPU->Low power mode settings
//                                    -Allowed low power modes: all modes allowed
//                                    -LLWU settings->Settings 
//                                           - External Pin 15:  Any edge(to generate wake up when SW3 is pressed)
//                                           - Internal module 0(LPTMR0): Enabled
//                                           - all other Internal modules and Input Filter must be disabled  
//                                    -Interrupts 
//                                           - Interrupt request : Disabled   
//                                           - Interrupt priority: 2
//                                    -Operation mode settings 
//                                           - WAIT operation mode:
//                                                         - Return to wait after ISR: no  
//                                           - SLEEP operation mode:
//                                                         - Return to stop after ISR: no  
//                                           - STOP operation mode: Enabled
//                                                         - Low power mode: LLS  
//*****************************************************************************
//*****************************************************************************
//   2: MCU/Radio low power modes:
//        MCU in VLPS mode.
//        Radio in sleep mode.
//      Wakeup sources:
//        GPIO (push button) interrupt(SW1).
//        UART interrupt.
//        LPTMR interrupt.
//          - LPTMR wakeup timeout: fixed at compile time.
//          - LPTMR clock source:   LPO(internal low power oscillator) calibrated versus Phy Timer period
//          - LPTMR resolution:     LPO period(prescaler = 1)
//                                  
//          - Deep sleep timeout:   cPWR_DeepSleepDurationMs by default. Use PWR_SetDeepSleepTimeInMs or 
//                                  to change it at run time. MAC can set an absolute time at which the system must be up.
//      PEx settings: CPU->Low power mode settings
//                                    -Allowed low power modes: all modes allowed
//                                    -LLWU settings->Settings: NA 
//                                    -Interrupts : NA
//                                    -Operation mode settings 
//                                           - WAIT operation mode:
//                                                         - Return to wait after ISR: no  
//                                           - SLEEP operation mode:
//                                                         - Return to stop after ISR: no  
//                                           - STOP operation mode: Enabled
//                                                         - Low power mode: NA(VLPS mode doesn't exist for now).   
//*****************************************************************************

#ifndef cPWR_DeepSleepMode
  #define cPWR_DeepSleepMode                     1
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
#define cPWR_LPTMRClockSource                   LPTMR_PDD_SOURCE_LPO1KHZ

//-----------------------------------------------------------------------------
// The deep sleep duration in ms. 
#ifndef cPWR_DeepSleepDurationMs
  #define cPWR_DeepSleepDurationMs                3000
#endif

//-----------------------------------------------------------------------------
// Enabling of external call to a procedure each time that DeepSleep are exited
//   0: Don't call any functions after DeepSleep (MAC)
//   1: Call a function after DeepSleep (Stack)
#ifndef cPWR_CallWakeupStackProcAfterDeepSleep
  #define cPWR_CallWakeupStackProcAfterDeepSleep  FALSE
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
  #if (gTMR_Enabled_d != TRUE) 
    #error "*** ERROR: Illegal value in cPWR_LVD_Enable"
  #endif
#endif

#if (cPWR_UsePowerDownMode > 1)
  #error "*** ERROR: Illegal value in cPWR_UsePowerDownMode"
#endif

#if (cPWR_CallWakeupStackProcAfterDeepSleep > 1)
  #error "*** ERROR: Illegal value in cPWR_CallWakeupStackProcAfterDeepSleep"
#endif

#if (cPWR_DeepSleepMode > 2)
  #error "*** ERROR: Illegal value in cPWR_DeepSleepMode"
#endif

#if (cPWR_SleepMode > 1)
  #error "*** ERROR: Illegal value in cPWR_SleepMode"
#endif

#endif /* _PWR_CONFIGURATION_H_ */
