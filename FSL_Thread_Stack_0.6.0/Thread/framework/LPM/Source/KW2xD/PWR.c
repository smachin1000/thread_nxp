/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PWR.c
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

/*****************************************************************************
 *                               INCLUDED HEADERS                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the headers that this module needs to include.    *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
#include "EmbeddedTypes.h"
#include "PWR_Configuration.h"
#include "PWRLib.h"
#include "PWR_Interface.h"
#include "TimersManager.h"
#include "Phy.h"
#include "MCR20Drv.h"
#include "MCR20Reg.h"
#include "Keyboard.h"
#include "SerialManager.h"

#include "fsl_lptmr_hal.h"
#include "fsl_smc_hal.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

#if (cPWR_UsePowerModuleStandAlone == 0)
#include "MacInterface.h"
#endif
/*****************************************************************************
 *                             PRIVATE MACROS                                *
 *---------------------------------------------------------------------------*
 * Add to this section all the access macros, registers mappings, bit access *
 * macros, masks, flags etc ...                                              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/* Minimum sleep ticks (16us) in DeepSleepMode 13  */
#define PWR_MINIMUM_SLEEP_TICKS   10
#define gPhyTimerFreq_c           62500  
#define gRTCOscFreq_c             32768  
#define gClockConfig_FEI_24_c   CPU_CLOCK_CONFIG_0
#define gClockConfig_PEE_48_c   CPU_CLOCK_CONFIG_1   
#define gClockConfig_BLPI_4_c   CPU_CLOCK_CONFIG_2      
   
/*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
uint8_t mLPMFlag = gAllowDeviceToSleep_c;

#if (cPWR_UsePowerDownMode)
static uint32_t mPWR_DeepSleepTime = cPWR_DeepSleepDurationMs;
static bool_t   mPWR_DeepSleepTimeInSymbols = FALSE;
static smc_power_mode_config_t mPWR_SmcConfig = {
    .lpwuiOption = 1,
    .lpwuiOptionValue = kSmcLpwuiEnabled,
};

const clock_manager_user_config_t mPWR_ClockConfig[] = 
{
    /* Configuration for enter VLPR mode. Core clock = 4MHz. */
    {
        .mcgConfig =
        {
            .mcg_mode           = kMcgModeBLPI,   // Work in BLPI mode.
            .irclkEnable        = true,  // MCGIRCLK enable.
            .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
            .ircs               = kMcgInternalRefClkSelFast, // Select IRC4M.
            .fcrdiv             = 0U,    // FCRDIV is 0.
            
            .frdiv   = 0U,
            .drs     = kMcgDcoRangeSelLow,  // Low frequency range
            .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
            .oscsel  = kMcgOscselOsc,       // Select OSC
            
            .pll0Enable        = false,  // PLL0 disable
            .pll0EnableInStop  = false,  // PLL0 disalbe in STOP mode
            .prdiv0            = 0U,
            .vdiv0             = 0U,
        },
        .simConfig =
        {
            .PllFllSel = kClockPllFllSelPll,    // PLLFLLSEL select PLL.
            .er32kSrc  = kClockEr32kSrcRtc,     // ERCLK32K selection, use RTC.
            .outdiv1   = 0U,
            .outdiv2   = 0U,
            .outdiv3   = 0U,
            .outdiv4   = 4U,
        },
        .oscerConfig =
        {
            .Enable       = true,  // OSCERCLK enable.
            .EnableInStop = false, // OSCERCLK disable in STOP mode.
        }
    },
    /* Configuration for enter RUN mode. Core clock = 48MHz. */
    {
        .mcgConfig =
        {
            .mcg_mode           = kMcgModePEE,   // Work in PEE mode.
            .irclkEnable        = false,  // MCGIRCLK enable.
            .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
            .ircs               = kMcgInternalRefClkSelSlow, // Select IRC32k.
            .fcrdiv             = 0U,    // FCRDIV is 0.

            .frdiv   = 5U,
            .drs     = kMcgDcoRangeSelLow,  // Low frequency range
            .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
            .oscsel  = kMcgOscselOsc,       // Select OSC

            .pll0Enable        = false,  // PLL0 disable
            .pll0EnableInStop  = false,  // PLL0 disalbe in STOP mode
            .prdiv0            = 0x01U,
            .vdiv0             = 0x00U,
        },
        .simConfig =
        {
            .PllFllSel = kClockPllFllSelPll,    // PLLFLLSEL select PLL.
            .er32kSrc  = kClockEr32kSrcRtc,     // ERCLK32K selection, use RTC.
            .outdiv1   = 0U,
            .outdiv2   = 0U,
            .outdiv3   = 1U,
            .outdiv4   = 1U,
        },
        .oscerConfig =
        {
            .Enable       = true,  // OSCERCLK enable.
            .EnableInStop = true, // OSCERCLK disable in STOP mode.
        }
    },
//    {
//        .mcgConfig =
//        {
//            .mcg_mode           = kMcgModeFEI,
//            .irclkEnable        = true,  // MCGIRCLK enable.
//            .irclkEnableInStop  = false, // MCGIRCLK disable in STOP mode.
//            .ircs               = kMcgInternalRefClkSelSlow,
//            .fcrdiv             = 0U,    // FCRDIV is 0.
//            
//            .frdiv   = 0U,
//            .drs     = kMcgDcoRangeSelLow,  // Low frequency range
//            .dmx32   = kMcgDmx32Default,    // DCO has a default range of 25%
//            .oscsel  = kMcgOscselOsc,       // Select OSC
//            
//            .pll0Enable        = false,  // PLL0 disable
//            .pll0EnableInStop  = false,  // PLL0 disalbe in STOP mode
//            .prdiv0            = 0U,
//            .vdiv0             = 0U,
//        },
//        .simConfig =
//        {
//            .PllFllSel = kClockPllFllSelFll, 
//            .er32kSrc  = kClockEr32kSrcLpo,
//            .outdiv1   = 0U,
//            .outdiv2   = 0U,
//            .outdiv3   = 0U,
//            .outdiv4   = 0U,
//        },
//        .oscerConfig =
//        {
//            .Enable       = true,  // OSCERCLK enable.
//            .EnableInStop = false, // OSCERCLK disable in STOP mode.
//        }
//    }
};
#endif //(cPWR_UsePowerDownMode)

/*****************************************************************************
 *                               PUBLIC VARIABLES                            *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have global      *
 * (project) scope.                                                          *
 * These variables / constants can be accessed outside this module.          *
 * These variables / constants shall be preceded by the 'extern' keyword in  *
 * the interface header.                                                     *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*****************************************************************************
 *                           PRIVATE FUNCTIONS PROTOTYPES                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
typedef enum 
{
  PWR_Run = 77,
  PWR_Sleep,
  PWR_DeepSleep,
  PWR_Reset,
  PWR_OFF
} PWR_CheckForAndEnterNewPowerState_t;
/*****************************************************************************
 *                             PRIVATE FUNCTIONS                             *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have local (file) scope.       *
 * These functions cannot be accessed outside this module.                   *
 * These definitions shall be preceded by the 'static' keyword.              *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/*****************************************************************************
 *                             PUBLIC FUNCTIONS                              *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions that have global (project) scope.   *
 * These functions can be accessed outside this module.                      *
 * These functions shall have their declarations (prototypes) within the     *
 * interface header file and shall be preceded by the 'extern' keyword.      *
 *---------------------------------------------------------------------------*
 *****************************************************************************/
 
/*---------------------------------------------------------------------------
 * Name: PWR_CheckForAndEnterNewPowerState_Init
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_CheckForAndEnterNewPowerState_Init
(
  void
)
{
#if (cPWR_UsePowerDownMode)

  PWRLib_Init();

#endif  /* #if (cPWR_UsePowerDownMode) */
}

/*---------------------------------------------------------------------------
 * Name: PWR_SetDeepSleepTimeInMs
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_SetDeepSleepTimeInMs
(
  uint32_t deepSleepTimeTimeMs
)
{
#if (cPWR_UsePowerDownMode)
 if(deepSleepTimeTimeMs == 0) 
 {
  return;
 }
 mPWR_DeepSleepTime = deepSleepTimeTimeMs;
 mPWR_DeepSleepTimeInSymbols = FALSE;
#else
 (void) deepSleepTimeTimeMs;
#endif
}

/*---------------------------------------------------------------------------
 * Name: PWR_SetDeepSleepTimeInSymbols
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_SetDeepSleepTimeInSymbols
(
  uint32_t deepSleepTimeTimeSym
)
{
#if (cPWR_UsePowerDownMode)
 if(deepSleepTimeTimeSym == 0) 
 {
  return;
 }
 mPWR_DeepSleepTime = deepSleepTimeTimeSym;
 mPWR_DeepSleepTimeInSymbols = TRUE;
#else
 (void) deepSleepTimeTimeSym;
#endif 
}

/*---------------------------------------------------------------------------
 * Name: PWR_AllowDeviceToSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_AllowDeviceToSleep
(
void
)
{
  OSA_EnterCritical(kCriticalDisableInt);
  
  if( mLPMFlag != 0 ){    
    mLPMFlag--;
  }
  OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
 * Name: PWR_DisallowDeviceToSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_DisallowDeviceToSleep
(
void
)
{
  uint8_t prot;
  OSA_EnterCritical(kCriticalDisableInt);
  prot = mLPMFlag + 1;
  if(prot != 0)
  {
    mLPMFlag++;
  }
  OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
 * Name: PWR_CheckIfDeviceCanGoToSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t PWR_CheckIfDeviceCanGoToSleep
(
void
)
{
  bool_t   returnValue;
  OSA_EnterCritical(kCriticalDisableInt);
  returnValue = mLPMFlag == 0 ? TRUE : FALSE;
  OSA_ExitCritical(kCriticalDisableInt);
  return returnValue;
}

/*---------------------------------------------------------------------------
 * Name: PWR_SleepAllowed
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t PWR_SleepAllowed
(
void
)
{
#if (cPWR_UsePowerDownMode)
#if (cPWR_UsePowerModuleStandAlone == 1)
  return TRUE;
#else  
  if((PWRLib_GetCurrentZigbeeStackPowerState == StackPS_Sleep) ||  \
    (PWRLib_GetCurrentZigbeeStackPowerState == StackPS_DeepSleep) )
  {
    if((PWRLib_GetMacStateReq()== gMacStateNotEmpty_c) || \
      (PWRLib_GetMacStateReq()== gMacStateIdle_c) )
    {
      return TRUE;
    }
    else
    {
      return FALSE;
    }
  }
  else
  {
    return FALSE;
  }
#endif //#if (cPWR_UsePowerModuleStandAlone)
#else
  return TRUE;
#endif  /* #if (cPWR_UsePowerDownMode) else */
}

/*---------------------------------------------------------------------------
 * Name: PWR_SleepAllowed
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
bool_t PWR_DeepSleepAllowed
(
void
)
{
#if (cPWR_UsePowerDownMode)
#if (cPWR_UsePowerModuleStandAlone == 1)
  return TRUE;
#else
  if (PWRLib_GetCurrentZigbeeStackPowerState == StackPS_DeepSleep) {
#if (cPWR_DeepSleepMode != 13)
    /* DeepSleepMode 13 allows the radio to be active during low power */
    if ( PWRLib_GetMacStateReq() == gMacStateIdle_c)
    {
      return TRUE;
    }
    else
    {
      return FALSE;
    }
#else /* #if (cPWR_DeepSleepMode != 13) */
    return TRUE;
#endif /* #if (cPWR_DeepSleepMode != 13) */
  }
  else
  {
    return FALSE;
  }
#endif //#if (cPWR_UsePowerModuleStandAlone)
#else
  return TRUE;
#endif  /* #if (cPWR_UsePowerDownMode)*/
}


/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
 
#if (cPWR_UsePowerDownMode)
static PWRLib_WakeupReason_t PWR_HandleDeepSleep
(
uint32_t DozeDuration
)
{
  PWRLib_WakeupReason_t  Res;
  uint32_t deepSleepTicks = 0;
#if ( (cPWR_DeepSleepMode == 9) || (cPWR_DeepSleepMode == 10) || (cPWR_DeepSleepMode == 11) || (cPWR_DeepSleepMode == 12) || (cPWR_DeepSleepMode == 13))
#if (gTMR_EnableLowPowerTimers_d)   
  uint32_t notCountedTicksBeforeSleep= 0;
#endif  
#endif
#if (cPWR_UsePowerDownMode)
  
  
  /* to avoid unused warning*/
  Res.AllBits = 0xff | (uint8_t) DozeDuration | (uint8_t) deepSleepTicks | (uint8_t) mPWR_DeepSleepTime | (uint8_t) mPWR_DeepSleepTimeInSymbols ;
  Res.AllBits = 0;
  PWRLib_MCU_WakeupReason.AllBits = 0;
  /*---------------------------------------------------------------------------*/
#if (cPWR_DeepSleepMode == 0)
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  Res.AllBits = 0xff | (uint8_t) DozeDuration;  // Last part to avoid unused warning
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 1)
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub2;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 2)
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Int_LPO_1KHz)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Int_LPO_1KHz"
#endif 
  //       /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cPWR_LPTMRTickTime, DozeDuration);

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub2;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  PWRLib_LPTMR_ClockStop();
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 3)
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Ext_ERCLK32K)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Ext_ERCLK32K"
#endif 
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cPWR_LPTMRTickTime, DozeDuration);

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub1;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  PWRLib_LPTMR_ClockStop();
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 4)
#if (!gTimestamp_Enabled_d)
#error "*** ERROR: gTimestamp_Enabled_d has to be set TRUE"
#endif 
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  TMR_RTCSetAlarmRelative(DozeDuration, NULL, NULL);

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub2;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits; 
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 5)
  
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Int_LPO_1KHz)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Int_LPO_1KHz"
#endif 
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cPWR_LPTMRTickTime, DozeDuration);

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub2;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  PWRLib_LPTMR_ClockStop();
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 6)
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Ext_ERCLK32K)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Ext_ERCLK32K"
#endif 
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cPWR_LPTMRTickTime, DozeDuration);

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub2;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  PWRLib_LPTMR_ClockStop();
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 7)
#if (!gTimestamp_Enabled_d)
#error "*** ERROR: gTimestamp_Enabled_d has to be set TRUE"
#endif /* #if (!gTimestamp_Enabled_d) */           
  
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  TMR_RTCSetAlarmRelative(DozeDuration, NULL, NULL);

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub2;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits; 
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 8)
  
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Int_LPO_1KHz)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Int_LPO_1KHz"
#endif 
  /* converts deep sleep duration from ms/symbols in LPTMR ticks */
  if(!mPWR_DeepSleepTimeInSymbols)
  {
    deepSleepTicks = ( mPWR_DeepSleepTime >> 1);
  }
  else
  {
    deepSleepTicks = ( mPWR_DeepSleepTime / 125 );
  }
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cLPTMR_PRS_00002ms, deepSleepTicks);

  /* configure MCU in VLLS2 low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeVlls;
  mPWR_SmcConfig.stopSubMode = kSmcStopSub2;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* never returns. VLLSx wakeup goes through Reset sequence. */
  //but
  //  If an interrupt configured to wake up the MCU from VLLS occurs before or 
  //  during the VLLS entry sequence it prevents the system from entering low power 
  //  and bit STOPA from SMC_PMCTRL becomes set. In this case the function returns 
  //  the reasons that prevent it to enter low power.
  PWRLib_LLWU_UpdateWakeupReason();
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits; 
  PWRLib_LPTMR_ClockStop();
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 9)
  
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Int_LPO_1KHz)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Int_LPO_1KHz"
#endif 
  
#if (gTMR_EnableLowPowerTimers_d)
  /* if more low power timers are running, stop the hardware timer
  and save the spend time in ticks that wasn't counted.  */
  notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif
  /* This is the place where PWRLib_LPTMR_ClockStart should be called*/
  /* Unfortunately LPTMR is reset by CLOCK_SYS_SetConfiguration()*/
  /* The duration of CLOCK_SYS_SetConfiguration() will not be counted*/
  
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cPWR_LPTMRTickTime, DozeDuration);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  
  /* configure MCU in LLS low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeLls;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* checks sources of wakeup */
  PWRLib_LLWU_UpdateWakeupReason();
  /* configure Radio in autodoze mode */
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* configure MCG in PEE/FEE mode*/
  /* stop LPTMR */
  PWRLib_LPTMR_ClockStop();
  /* configure MCG in PLL Engaged External (PEE) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  
  /* This is the place where PWRLib_LPTMR_ClockStop should be called*/
  /* Unfortunately LPTMR is reset by CLOCK_SYS_SetConfiguration()*/
  /* The duration of CLOCK_SYS_SetConfiguration() will not be counted*/
  /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
  {
    uint32_t deepSleepDurationMs;
    
    /* Converts the DozeDuration from PWRLib timer ticks in ms */
    if (cPWR_LPTMRTickTime == cLPTMR_PRS_00001ms)
    {
        deepSleepDurationMs = PWRLib_LPTMR_ClockCheck();
    }
    else
    {
        deepSleepDurationMs =  PWRLib_LPTMR_ClockCheck();
        deepSleepDurationMs <<=  ( (cPWR_LPTMRTickTime >> 1) + 1 ); 
    }
    
    /* Converts the DozeDuration from ms in software timer ticks and synchronize low power timers */
    TMR_SyncLpmTimers( (TmrTicksFromMilliseconds( ( tmrTimeInMilliseconds_t) deepSleepDurationMs) + notCountedTicksBeforeSleep) );
  }
#endif
  
  if(PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout == 1)
  {
    cPWR_DeepSleepWakeupStackProc; // User function called only on timeout
  }
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  
  
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 10)
  
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Ext_ERCLK32K)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Ext_ERCLK32K"
#endif /* #if (cPWR_LPTMRClockSource == cLPTMR_Source_Int_LPO_1KHz) */
  
  
#if (gTMR_EnableLowPowerTimers_d)
  /* if more low power timers are running, stop the hardware timer
  and save the spend time in ticks that wasn't counted.  */
  notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif 
  /* This is the place where PWRLib_LPTMR_ClockStart should be called*/
  /* Unfortunately LPTMR is reset by CLOCK_SYS_SetConfiguration()*/
  /* The duration of CLOCK_SYS_SetConfiguration() will not be counted*/
  
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cPWR_LPTMRTickTime, DozeDuration);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();
  /* configure MCU in LLS low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeLls;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);

  /* checks sources of wakeup */
  PWRLib_LLWU_UpdateWakeupReason();
  /* configure Radio in autodoze mode */
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* stop LPTMR */
  PWRLib_LPTMR_ClockStop();
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /* This is the place where PWRLib_LPTMR_ClockStop should be called*/
  /* Unfortunately LPTMR is reset by CLOCK_SYS_SetConfiguration()*/
  /* The duration of CLOCK_SYS_SetConfiguration() will not be counted*/
  
  /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
  {
    uint32_t lptmrFreq = gRTCOscFreq_c;
    uint64_t timerTicks;
    /* Converts the DozeDuration from PWRLib timer ticks in ms */
    if( cPWR_LPTMRTickTime != cLPTMR_PRS_125_div_by_4096ms )
    {
        lptmrFreq >>= ((cPWR_LPTMRTickTime >> 1) + 1);
    }
    timerTicks = (PWRLib_LPTMR_ClockCheck()*TMR_GetTimerFreq())/lptmrFreq;
    timerTicks += notCountedTicksBeforeSleep;
    if(timerTicks > 0xffffffff)
    {
      timerTicks = 0xffffffff;
    }
    TMR_SyncLpmTimers((uint32_t)timerTicks);
  }
#endif
  
  
  if( PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout)
  {
    cPWR_DeepSleepWakeupStackProc; // User function called only on timeout
  }
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 11)
#if (!gTimestamp_Enabled_d)
#error "*** ERROR: gTimestamp_Enabled_d has to be set TRUE"
#endif /* #if (!gTimestamp_Enabled_d) */           
  {  
    
#if (gTMR_EnableLowPowerTimers_d)
    uint64_t timeStamp;  
    /* if more low power timers are running, stop the hardware timer
    and save the spend time in ticks that wasn't counted.  */
    notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
    timeStamp = TMR_RTCGetTimestamp();
#endif  
    /* configure MCG in FLL Engaged Internal (FEI) mode */
    CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
    /* disable transceiver CLK_OUT. */
    MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
    /* configure Radio in hibernate mode */
    PWRLib_Radio_Enter_Hibernate();
    TMR_RTCSetAlarmRelative(DozeDuration, NULL, NULL);

    /* configure MCU in LLS low power mode */
    mPWR_SmcConfig.powerModeName = kPowerModeLls;
    SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
    
    /* checks sources of wakeup */
    PWRLib_LLWU_UpdateWakeupReason();
    /* configure Radio in autodoze mode */
    PWRLib_Radio_Enter_AutoDoze();
    MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
    CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
    /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
    {
      uint64_t timerTicks = ((TMR_RTCGetTimestamp() - timeStamp)*TMR_GetTimerFreq())/1000000;
      timerTicks += notCountedTicksBeforeSleep;
      if(timerTicks > 0xffffffff)
      {
        timerTicks = 0xffffffff;
      }
      TMR_SyncLpmTimers((uint32_t)timerTicks);
    }
#endif
    if(PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout == 1)
    {
      cPWR_DeepSleepWakeupStackProc; // User function called only on timeout
    }
    Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  }   
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 12)
  
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Int_LPO_1KHz)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Int_LPO_1KHz"
#endif 
  
  /* converts deep sleep duration from ms/symbols in LPTMR ticks */
  //LPTMR resolution:     fixed to 2 ms (125 802.15.4 PHY symbols)
  if(!mPWR_DeepSleepTimeInSymbols)
  {
    deepSleepTicks = ( mPWR_DeepSleepTime >>  1 );
  }
  else
  {
    deepSleepTicks = ( mPWR_DeepSleepTime / 125 );
  }
#if (gTMR_EnableLowPowerTimers_d)
  /* if more low power timers are running, stop the hardware timer
  and save the spend time in ticks that wasn't counted.  */
  notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif 
  
  /* This is the place where PWRLib_LPTMR_ClockStart should be called*/
  /* Unfortunately LPTMR is reset by CLOCK_SYS_SetConfiguration()*/
  /* The duration of CLOCK_SYS_SetConfiguration() will not be counted*/
  
  /* configure MCG in FLL Engaged Internal (FEI) mode */
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
  /* start LPTMR */
  PWRLib_LPTMR_ClockStart(cLPTMR_PRS_00002ms, deepSleepTicks);
  /* disable transceiver CLK_OUT. */
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
  /* configure Radio in hibernate mode */
  PWRLib_Radio_Enter_Hibernate();

  /* configure MCU in LLS low power mode */
  mPWR_SmcConfig.powerModeName = kPowerModeLls;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  /* checks sources of wakeup */
  PWRLib_LLWU_UpdateWakeupReason();
  /* configure Radio in autodoze mode */
  PWRLib_Radio_Enter_AutoDoze();
  MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
  /* stop LPTMR */
  PWRLib_LPTMR_ClockStop();
  /* configure MCG in PEE/FEE mode*/
  CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
  /* This is the place where PWRLib_LPTMR_ClockStop should be called*/
  /* Unfortunately LPTMR is reset by CLOCK_SYS_SetConfiguration()*/
  /* The duration of CLOCK_SYS_SetConfiguration() will not be counted*/
  
  /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
  {
    uint32_t deepSleepDurationMs;
    /* Converts the DozeDuration from PWRLib timer ticks in ms */
    deepSleepDurationMs = ( PWRLib_LPTMR_ClockCheck() << 1 );
    /* Converts the DozeDuration from ms in software timer ticks and synchronize low power timers */
    TMR_SyncLpmTimers( (TmrTicksFromMilliseconds( ( tmrTimeInMilliseconds_t) deepSleepDurationMs) + notCountedTicksBeforeSleep) );
  }
#endif
  if(PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout == 1)
  {
    cPWR_DeepSleepWakeupStackProc; // User function called only on timeout
  }
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  
  
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 13)
#if (!gKeyBoardSupported_d)
#error "*** ERROR: gKeyBoardSupported_d has to be set to TRUE"
#endif 
  
#if ( gSerialMgrUseUart_c  == 0 )
#error "*** ERROR: gSerialMgrUseUart_c has to be set to TRUE"
#endif            
  {
    
    /* converts deep sleep duration from ms to symbols */
    if(!mPWR_DeepSleepTimeInSymbols)
    {
      deepSleepTicks = ( ( ( mPWR_DeepSleepTime / 2 ) * 125 ) + ( ( mPWR_DeepSleepTime & 1 ) * 62 ) ) & 0xFFFFFF;
    }
    else
    {
      deepSleepTicks = mPWR_DeepSleepTime;
    }

    if( deepSleepTicks > PWR_MINIMUM_SLEEP_TICKS )
    {
#if (gTMR_EnableLowPowerTimers_d)      
      uint32_t lptSyncTime; 
#endif      
      uint32_t currentTime;
      uint32_t absoluteWakeUpTime;
      uint32_t temp;
      // Set prescaller to obtain 1 symbol (16us) timebase TODO
      MCR20Drv_IndirectAccessSPIWrite(TMR_PRESCALE, 0x05);
#if (gTMR_EnableLowPowerTimers_d)
      /* if more low power timers are running, stop the hardware timer
      and save the spend time in ticks that wasn't counted.  */
      notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
      PhyTimeReadClock(&lptSyncTime);
#endif

      /* configure MCG in FLL Engaged Internal (BLPI) mode */
      CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
      /* disable SysTick counter and interrupt */
      temp = SysTick->CTRL & (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
      SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
      /* disable transceiver CLK_OUT. */
      MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
      /* configure Radio in Doze mode */
      PWRLib_Radio_Enter_Doze();
      /* prepare UART for low power operation */
      (void)Serial_EnableLowPowerWakeup(gSerialMgrUart_c);
      /* read current time */
      PhyTimeReadClock(&currentTime);
      /* compute absolute end time */
      absoluteWakeUpTime = (uint32_t)((currentTime + deepSleepTicks) & 0xFFFFFF);
      /* set absolute wakeup time */
      PhyTimeSetWakeUpTime(&absoluteWakeUpTime);

      /* configure MCU in VLPS low power mode */
      mPWR_SmcConfig.powerModeName = kPowerModeVlps;
      SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
      /* checks sources of wakeup */
      /* radio timer wakeup */
      if( PhyTimeIsWakeUpTimeExpired() )
      {
        PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout = 1;     /* Sleep timeout ran out */
        PWRLib_MCU_WakeupReason.Bits.FromTimer = 1;            /* Wakeup by radio timer */
      }
      if( KBD_IsWakeUpSource() )
      {
        PWRLib_MCU_WakeupReason.Bits.FromKeyBoard = 1;
      }
      
      /*Check  UART module wakeup */
      if(Serial_IsWakeUpSource(gSerialMgrUart_c))
      {
        PWRLib_MCU_WakeupReason.Bits.FromUART = 1;
      }
      (void)Serial_DisableLowPowerWakeup( gSerialMgrUart_c);        
      /* configure Radio in autodoze mode */
      PWRLib_Radio_Enter_AutoDoze();
      MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);
      /* configure MCG in PEE/FEE mode*/
      CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);
      /* restore the state of SysTick */
      SysTick->CTRL |= temp;
      
#if (gTMR_EnableLowPowerTimers_d)
      {
        uint64_t timerTicks;
        PhyTimeReadClock((uint32_t*)&absoluteWakeUpTime);
        /* Converts the DozeDuration from radio ticks in software timer ticks and synchronize low power timers */
        timerTicks = (((absoluteWakeUpTime - lptSyncTime)& 0xFFFFFF)*TMR_GetTimerFreq())/gPhyTimerFreq_c;
        timerTicks += notCountedTicksBeforeSleep;
        if(timerTicks > 0xffffffff)
        {
          timerTicks = 0xffffffff;
        }
        TMR_SyncLpmTimers( (uint32_t)timerTicks );
      } 
#endif
    }
    else
    {
      /* Not enough time to program the TRM compare */
      PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout = 1;     /* Sleep timeout ran out */
    }
    if(Res.Bits.DeepSleepTimeout == 1) 
    {
      cPWR_DeepSleepWakeupStackProc; // User function called only on timeout
    }
    Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
    
  }
  
    /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 14)
#if (!gKeyBoardSupported_d)
#error "*** ERROR: gKeyBoardSupported_d has to be set to TRUE"
#endif

    /* configure MCG in FLL Engaged Internal (FEI) mode */
    CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);

    /* disable transceiver CLK_OUT. */
    MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);

    /* configure Radio in hibernate mode */
    PWRLib_Radio_Enter_Hibernate();
    
    #if gPWR_EnsureOscStabilized_d
      /* start 32KHz OSC */
      while(PWRLib_RTC_IsOscStarted() == FALSE){}
    #endif

    /* configure MCU in LLS low power mode */
    mPWR_SmcConfig.powerModeName = kPowerModeLls;
    SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);

    /* checks sources of wakeup */
    PWRLib_LLWU_UpdateWakeupReason();
    
    /* GPIO wakeup */
    if( KBD_IsWakeUpSource() )
    {
      PWRLib_MCU_WakeupReason.Bits.FromKeyBoard = 1;
    }
                    
    /* configure Radio in autodoze mode */
    PWRLib_Radio_Enter_AutoDoze();
    
    MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_4_MHz);

    /* configure MCG in PEE/FEE mode*/
    CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[1]);

    Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;      
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 15)
#if (!gKeyBoardSupported_d)
#error "*** ERROR: gKeyBoardSupported_d has to be set to TRUE"
#endif 

    /* configure MCU in LLS low power mode */
    mPWR_SmcConfig.powerModeName = kPowerModeLls;
    SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);

    /* checks sources of wakeup */
    PWRLib_LLWU_UpdateWakeupReason();

    /* GPIO wakeup */
    if( KBD_IsWakeUpSource() )
    {
      PWRLib_MCU_WakeupReason.Bits.FromKeyBoard = 1;
    }

    Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;   

  /*---------------------------------------------------------------------------*/
#else
#error "*** ERROR: Not a valid cPWR_DeepSleepMode chosen"
#endif
  
  return Res;
  
#else  /* #if (cPWR_UsePowerDownMode) else */
  
  /* to avoid unused warning*/
  Res.AllBits = 0xff | (uint8_t) DozeDuration | (uint8_t) deepSleepTicks;
  
  PWRLib_MCU_WakeupReason.AllBits = 0;
  return Res;          /* (PWRLib_WakeupReason_t) DozeDuration; */
#endif  /* #if (cPWR_UsePowerDownMode) end */
  
}
#endif /* cPWR_UsePowerDownMode */

/*---------------------------------------------------------------------------
 * Name: PWR_HandleSleep
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_WakeupReason_t PWR_HandleSleep
(
uint32_t DozeDuration
)
{
  PWRLib_WakeupReason_t  Res;
  
  Res.AllBits = 0;
  
  (void) DozeDuration;
#if (cPWR_UsePowerDownMode)
  /*---------------------------------------------------------------------------*/
#if (cPWR_SleepMode==0)
  return Res;
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_SleepMode==1)
  /* radio in autodoze mode by default. mcu in wait mode */
  PWRLib_MCU_WakeupReason.AllBits = 0;
  mPWR_SmcConfig.powerModeName = kPowerModeWait;
  SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
  Res.Bits.SleepTimeout = 1;
  PWRLib_MCU_WakeupReason.Bits.SleepTimeout = 1;
  return Res;
  /*---------------------------------------------------------------------------*/
#else
#error "*** ERROR: Not a valid cPWR_SleepMode chosen"
#endif
#else  /* #if (cPWR_UsePowerDownMode) else */
  /* Last part to avoid unused warning */
  PWRLib_MCU_WakeupReason.AllBits = 0;
  return Res;          /* (PWRLib_WakeupReason_t) DozeDuration */
#endif  /* #if (cPWR_UsePowerDownMode) end */
}

/*---------------------------------------------------------------------------
 * Name: PWR_CheckForAndEnterNewPowerState
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_WakeupReason_t PWR_CheckForAndEnterNewPowerState
(
PWR_CheckForAndEnterNewPowerState_t NewPowerState,
uint32_t DozeDuration
)
{
  PWRLib_WakeupReason_t ReturnValue;
  ReturnValue.AllBits = 0;
  
#if (cPWR_UsePowerDownMode)
  if ( NewPowerState == PWR_Run)
  {
    /* ReturnValue = 0; */
  }
  else if( NewPowerState == PWR_OFF)
  {
    /* configure MCG in FLL Engaged Internal (FEI) mode */
    CLOCK_SYS_SetConfiguration(&mPWR_ClockConfig[0]);
    /* puts radio RESET */
    // COCO_RST_B_ClrVal(NULL);
    MCR20Drv_Set_CLK_OUT_Freq(gCLK_OUT_FREQ_DISABLE);
    /* configure Radio in hibernate mode */
    PWRLib_Radio_Enter_Hibernate();
    // disable all wake up sources
    LLWU_PE1 = 0;
    LLWU_PE2 = 0;
    LLWU_PE3 = 0;
    LLWU_PE4 = 0;
    LLWU_ME = 0;
    /* configure MCU in VLLS1 mode */
    mPWR_SmcConfig.powerModeName = kPowerModeVlls;
    mPWR_SmcConfig.stopSubMode = kSmcStopSub1;
    SMC_HAL_SetMode(SMC_BASE, &mPWR_SmcConfig);
    
    /* Never returns */
    for(;;){}

  }
  else if( NewPowerState == PWR_Reset)
  {
    /* Never returns */
    PWRLib_Reset();
  }
  
  else if(( NewPowerState == PWR_DeepSleep) && PWR_DeepSleepAllowed())
  {
    ReturnValue = PWR_HandleDeepSleep(DozeDuration);
  } 
  else if(( NewPowerState == PWR_Sleep) && PWR_SleepAllowed())
  {
    ReturnValue = PWR_HandleSleep(DozeDuration);
  }
  else
  {
    /* ReturnValue = FALSE; */
  }
  /* Clear wakeup reason */
  
#else
  /* To remove warning for variabels in functioncall */
  (void)NewPowerState;
  (void)DozeDuration;
#endif  /* #if (cPWR_UsePowerDownMode) */
  
  return ReturnValue;
}

/*---------------------------------------------------------------------------
 * Name: PWR_EnterPowerOff
 * Description: - Radio on Reset, MCU on VLLS1
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_EnterPowerOff(void)
{
  OSA_EnterCritical(kCriticalDisableInt);
  (void)PWR_CheckForAndEnterNewPowerState(PWR_OFF,0);
  OSA_ExitCritical(kCriticalDisableInt);
}
/*---------------------------------------------------------------------------
 * Name: PWRLib_LVD_ReportLevel
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_LVD_VoltageLevel_t PWRLib_LVD_ReportLevel
(
void
)
{
  PWRLib_LVD_VoltageLevel_t   Level;
#if ((cPWR_LVD_Enable == 0) || (cPWR_LVD_Enable == 3))
  Level = PWR_ABOVE_LEVEL_3_0V;
#elif (cPWR_LVD_Enable==1)
  Level = PWRLib_LVD_CollectLevel();
#elif (cPWR_LVD_Enable==2)
  Level = PWRLib_LVD_SavedLevel;
#else
#error "*** ERROR: Illegal value for cPWR_LVD_Enable"
#endif /* #if (cPWR_LVD_Enable) */
  return Level;
}

/*---------------------------------------------------------------------------
 * Name: PWR_EnterLowPower
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_WakeupReason_t PWR_EnterLowPower
(
void
)
{
  PWRLib_WakeupReason_t ReturnValue;  
#if (gTMR_EnableLowPowerTimers_d) 
  bool_t unlockTMRThread = FALSE;
#endif
  ReturnValue.AllBits = 0;
  
  if (PWRLib_LVD_ReportLevel() == PWR_LEVEL_CRITICAL)
  {
    /* Voltage <= 1.8V so enter power-off state - to disable false Tx'ing(void)*/
    ReturnValue = PWR_CheckForAndEnterNewPowerState( PWR_OFF, 0);
  }

  INT_SYS_DisableIRQGlobal();
  
#if (cPWR_UsePowerModuleStandAlone == 0)
  PWRLib_SetCurrentZigbeeStackPowerState(StackPS_DeepSleep);
#endif
  if (
      TMR_AreAllTimersOff()
#if ( (cPWR_DeepSleepMode == 3) || (cPWR_DeepSleepMode == 4) || (cPWR_DeepSleepMode == 6) || (cPWR_DeepSleepMode == 7) || (cPWR_DeepSleepMode == 10) || (cPWR_DeepSleepMode == 11) )
#if gPWR_EnsureOscStabilized_d
        && TMR_RTCIsOscStarted()   
#endif
#endif          
          )  /*No timer running*/
  {
    /* if power lib is enabled */   
#if (cPWR_UsePowerDownMode)
    /* if Low Power Capability is enabled */
#if (gTMR_EnableLowPowerTimers_d) 
    /* if more low power timers are running, stop the hardware timer
    and save the spend time in ticks that wasn't counted.
    */
    unlockTMRThread = TRUE;
#endif /* #if (gTMR_EnableLowPowerTimers_d)  */
#endif /* #if (cPWR_UsePowerDownMode)  */

    ReturnValue = PWR_CheckForAndEnterNewPowerState (PWR_DeepSleep, cPWR_TMRTicks);
  }
  else /*timers are running*/
  {      
    ReturnValue = PWR_CheckForAndEnterNewPowerState (PWR_Sleep, 0);
  }

  /* enable irq's if there is pending evens */
  INT_SYS_EnableIRQGlobal();

#if (gTMR_EnableLowPowerTimers_d)
  if(unlockTMRThread)
  {
    TMR_MakeTMRThreadReady();
  }
#endif

  return ReturnValue;
}
