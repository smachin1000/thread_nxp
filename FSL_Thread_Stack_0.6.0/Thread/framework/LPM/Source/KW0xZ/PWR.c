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
#include "LPTMR.h"  
#include "PhyTypes.h"
#include "PhyTime.h"
#include "PWR_Configuration.h"
#include "PWRLib.h"
#include "PWR_Interface.h"
#include "TimersManager.h"
#include "SX123xDrv.h"
#include "Keyboard.h"
#include "Cpu.h"
#include "LPTMR.h"
#include "SystemTimer1.h"
#include "SerialManager.h"
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

#define gClockConfig_FEI_24_c   CPU_CLOCK_CONFIG_0
#define gClockConfig_PEE_48_c   CPU_CLOCK_CONFIG_1   
   
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
static phyTime_t mPWR_DeepSleepTimeInPhyTicks;      
static phyTime_t mPWR_AbsoluteWakeupTimeInPhyTicks;   
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
 * Name: PWR_SetAbsoluteWakeupTimeInPhyTicks
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWR_SetAbsoluteWakeupTimeInPhyTicks
(
  phyTime_t phyTicks
)
{
#if (cPWR_UsePowerDownMode)
 OSA_EXT_InterruptDisable();
 mPWR_AbsoluteWakeupTimeInPhyTicks = phyTicks;
 OSA_EXT_InterruptEnable();
#else
 (void) phyTicks;
#endif
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
 mPWR_DeepSleepTimeInPhyTicks = TIME_US_TO_TICKS( deepSleepTimeTimeMs*1000 ); 
 #else
 (void) deepSleepTimeTimeMs;
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
  OSA_EXT_InterruptDisable();
  
  if( mLPMFlag != 0 ){    
    mLPMFlag--;
  }
  OSA_EXT_InterruptEnable();
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
  OSA_EXT_InterruptDisable();
  prot = mLPMFlag + 1;
  if(prot != 0)
  {
    mLPMFlag++;
  }
  OSA_EXT_InterruptEnable();
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
  OSA_EXT_InterruptDisable();
  returnValue = mLPMFlag == 0 ? TRUE : FALSE;
  OSA_EXT_InterruptEnable();
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
  if (PWRLib_GetCurrentZigbeeStackPowerState == StackPS_DeepSleep)
  {
    if ( PWRLib_GetMacStateReq() == gMacStateIdle_c)
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
#endif  /* #if (cPWR_UsePowerDownMode)*/
}


/*---------------------------------------------------------------------------
 * Name: PWR_HandleDeepSleep
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/

static PWRLib_WakeupReason_t PWR_HandleDeepSleep
(
)
{
  PWRLib_WakeupReason_t  Res;
  
#if (cPWR_UsePowerDownMode)
#if ( (cPWR_DeepSleepMode == 1) || (cPWR_DeepSleepMode == 2))
#if (gTMR_EnableLowPowerTimers_d)   
  uint32_t notCountedTicksBeforeSleep= 0;
#endif  
#endif  
    
  Res.AllBits = 0;
  PWRLib_MCU_WakeupReason.AllBits = 0;
  /*---------------------------------------------------------------------------*/
#if (cPWR_DeepSleepMode == 0)
  (void)mPWR_DeepSleepTimeInPhyTicks;      
  (void)mPWR_AbsoluteWakeupTimeInPhyTicks;   
  
  /*---------------------------------------------------------------------------*/

#elif (cPWR_DeepSleepMode == 1)
  
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Int_LPO_1KHz)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Int_LPO_1KHz"
#endif 

  {
    phyTime_t currentTimeInPhyTicks;
    phyTime_t sleepTimeInPhyTicks;
    uint32_t sleepTimeInLPOTicks;
    PhyTimeReadClockTicks(&currentTimeInPhyTicks);  
    if(mPWR_AbsoluteWakeupTimeInPhyTicks > currentTimeInPhyTicks)
    {
      sleepTimeInPhyTicks = mPWR_AbsoluteWakeupTimeInPhyTicks - currentTimeInPhyTicks;
      if(sleepTimeInPhyTicks > mPWR_DeepSleepTimeInPhyTicks)
      {
        sleepTimeInPhyTicks = mPWR_DeepSleepTimeInPhyTicks;
      }
    }
    else
    {
      sleepTimeInPhyTicks = mPWR_DeepSleepTimeInPhyTicks;
    }
    sleepTimeInLPOTicks = PWRLib_LPTMR_PhyTicksToLPOTicks(sleepTimeInPhyTicks);
    
    if(sleepTimeInLPOTicks > 4)
    {
      
      sleepTimeInLPOTicks -= 3;/*When the LPTMR is enabled, the first
                               increment will take an additional one or two prescaler clock cycles due to
                               synchronization logic.At LP exit another tick is wait for precision */
      if(sleepTimeInLPOTicks > 0x10000)
      {
        sleepTimeInLPOTicks = 0x10000;
      }
      /* start LPTMR */
      PWRLib_LPTMR_ClockStart(cLPTMR_PRS_00001ms, sleepTimeInLPOTicks);
      while(PWRLib_LPTMR_GetCounterValue() == 0);
      PhyTimerStop();
#if (gTMR_EnableLowPowerTimers_d)
      /* if more low power timers are running, stop the hardware timer
      and save the spend time in ticks that wasn't counted.  */
      notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif
      /* configure MCG in FLL Engaged Internal (FEI) mode */      
      Cpu_SetClockConfiguration(gClockConfig_FEI_24_c);
      /* configure Radio in sleep mode */
      (void)MKW01Drv_RadioSleepReq();
      //  PORTA_PCR2 = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; //TODO
      Cpu_SetOperationMode(DOM_STOP,NULL,NULL);
      //while(LPTMR_PDD_GetInterruptFlag(LPTMR0_BASE_PTR)== 0);
      /* checks sources of wakeup */
      PWRLib_LLWU_UpdateWakeupReason();
      /* configure Radio in idle mode */
      MKW01Drv_RadioWakeUpReq();
      
      /* configure MCG in PEE/FEE mode*/
      Cpu_SetClockConfiguration(gClockConfig_PEE_48_c);
      {
        uint32_t lptmrValue = PWRLib_LPTMR_GetCounterValue();
        while( lptmrValue == PWRLib_LPTMR_GetCounterValue());
        
      }
      PhyTimerStart();
      /* stop LPTMR */
      PWRLib_LPTMR_ClockStop();
      sleepTimeInLPOTicks = PWRLib_LPTMR_ClockCheck() - 1;
      sleepTimeInPhyTicks = PWRLib_LPTMR_LPOTicksToPhyTicks(sleepTimeInLPOTicks);
      PhyTimeSyncClockTicks(sleepTimeInPhyTicks);
      /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
      {
        uint64_t deepSleepDurationUs;
        uint64_t timerTicks;
        deepSleepDurationUs = TIME_TICKS_TO_US(sleepTimeInPhyTicks);
        timerTicks = deepSleepDurationUs * TMR_GetTimerFreq() / 1000000;
        timerTicks += notCountedTicksBeforeSleep;
        TMR_SyncLpmTimers((uint32_t)timerTicks);
      }
#endif     
    }

  }
   
  if(PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout == 1)
  {
    cPWR_DeepSleepWakeupStackProc; // User function called only on timeout
  }
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;
  /*---------------------------------------------------------------------------*/
#elif (cPWR_DeepSleepMode == 2)
#if (cPWR_LPTMRClockSource != cLPTMR_Source_Int_LPO_1KHz)
#error  "*** ERROR: cPWR_LPTMRClockSource has to be set to cLPTMR_Source_Int_LPO_1KHz"
#endif 
#if (gKeyBoardSupported_d == FALSE)
#error "*** ERROR: gKeyBoardSupported_d has to be set to TRUE"
#endif 
  
#if ( (gSerialMgrUseUartA_c  || gSerialMgrUseUartB_c) == FALSE)
#error "*** ERROR: gSerialMgrUseUartA_c or gSerialMgrUseUartB_c has to be set to TRUE"
#endif            
  {
    phyTime_t currentTimeInPhyTicks;
    phyTime_t sleepTimeInPhyTicks;
    uint32_t sleepTimeInLPOTicks;
    PhyTimeReadClockTicks(&currentTimeInPhyTicks);  
    if(mPWR_AbsoluteWakeupTimeInPhyTicks > currentTimeInPhyTicks)
    {
      sleepTimeInPhyTicks = mPWR_AbsoluteWakeupTimeInPhyTicks - currentTimeInPhyTicks;
      if(sleepTimeInPhyTicks > mPWR_DeepSleepTimeInPhyTicks)
      {
        sleepTimeInPhyTicks = mPWR_DeepSleepTimeInPhyTicks;
      }
    }
    else
    {
      sleepTimeInPhyTicks = mPWR_DeepSleepTimeInPhyTicks;
    }
    sleepTimeInLPOTicks = PWRLib_LPTMR_PhyTicksToLPOTicks(sleepTimeInPhyTicks);
    
    if(sleepTimeInLPOTicks > 4)
    {
      
      sleepTimeInLPOTicks -= 3;/*When the LPTMR is enabled, the first
                               increment will take an additional one or two prescaler clock cycles due to
                               synchronization logic.*/
      if(sleepTimeInLPOTicks > 0x10000)
      {
        sleepTimeInLPOTicks = 0x10000;
      }
      /* start LPTMR */
      PWRLib_LPTMR_ClockStart(cLPTMR_PRS_00001ms, sleepTimeInLPOTicks);
      while(PWRLib_LPTMR_GetCounterValue() == 0);
      PhyTimerStop();
#if (gTMR_EnableLowPowerTimers_d)
      /* if more low power timers are running, stop the hardware timer
      and save the spend time in ticks that wasn't counted.  */
      notCountedTicksBeforeSleep = TMR_NotCountedTicksBeforeSleep();
#endif
      /* configure MCG in FLL Engaged Internal (FEI) mode */      
      Cpu_SetClockConfiguration(gClockConfig_FEI_24_c);
      SysTick_PDD_EnableDevice(SysTick_BASE_PTR, PDD_DISABLE);
      SysTick_PDD_ClearInterruptFlag(SysTick_BASE_PTR);      
      /* configure Radio in sleep mode */
      (void)MKW01Drv_RadioSleepReq();
      (void)Serial_EnableLowPowerWakeup(gSerialMgrUart_c);
      //  PORTA_PCR2 = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; //TODO
      PWRLib_MCU_Enter_VLPS();
      /* checks sources of wakeup */
      if( PWRLib_LPTMR_IsWakeUpTimeExpired() )
      {
        PWRLib_MCU_WakeupReason.Bits.FromLPTMR = 1;
        PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout = 1;
      }
      if( KBD_IsWakeUpSource() == TRUE)
      {
        PWRLib_MCU_WakeupReason.Bits.FromKeyBoard = 1;
      }
      
      /*Check  UART module wakeup */
      if(Serial_IsWakeUpSource(gSerialMgrUart_c))
      {
        PWRLib_MCU_WakeupReason.Bits.FromUART = 1;
      }
      (void)Serial_DisableLowPowerWakeup( gSerialMgrUart_c);        
      /* configure Radio in idle mode */
      MKW01Drv_RadioWakeUpReq();
      
      /* configure MCG in PEE/FEE mode*/
      Cpu_SetClockConfiguration(gClockConfig_PEE_48_c);
      SysTick_PDD_EnableDevice(SysTick_BASE_PTR, PDD_ENABLE);   
      {
        uint32_t lptmrValue = PWRLib_LPTMR_GetCounterValue();
        while( lptmrValue == PWRLib_LPTMR_GetCounterValue());
        
      }
      PhyTimerStart();
      /* stop LPTMR */
      PWRLib_LPTMR_ClockStop();
      sleepTimeInLPOTicks = PWRLib_LPTMR_ClockCheck() - 1;
      sleepTimeInPhyTicks = PWRLib_LPTMR_LPOTicksToPhyTicks(sleepTimeInLPOTicks);
      PhyTimeSyncClockTicks(sleepTimeInPhyTicks);

      /* Sync. the low power timers */
#if (gTMR_EnableLowPowerTimers_d)
      {
        uint64_t deepSleepDurationUs;
        uint64_t timerTicks;
        deepSleepDurationUs = TIME_TICKS_TO_US(sleepTimeInPhyTicks);
        timerTicks = deepSleepDurationUs * TMR_GetTimerFreq() / 1000000;
        timerTicks += notCountedTicksBeforeSleep;
        TMR_SyncLpmTimers((uint32_t)timerTicks);
      }
#endif     
    }

  }
   
  if(PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout == 1)
  {
    cPWR_DeepSleepWakeupStackProc; // User function called only on timeout
  }
  Res.AllBits = PWRLib_MCU_WakeupReason.AllBits;  
  /*---------------------------------------------------------------------------*/
#else
#error "*** ERROR: Not a valid cPWR_DeepSleepMode chosen"
#endif
  
  return Res;
  
#else  /* #if (cPWR_UsePowerDownMode) else */
  
  /* to avoid unused warning*/
  Res.AllBits = 0;
  PWRLib_MCU_WakeupReason.AllBits = 0;
  return Res;          
#endif  /* #if (cPWR_UsePowerDownMode) end */
  
}


/*---------------------------------------------------------------------------
 * Name: PWR_HandleSleep
 * Description: - 
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
PWRLib_WakeupReason_t PWR_HandleSleep
(
)
{
  PWRLib_WakeupReason_t  Res;
  
  Res.AllBits = 0;
    
#if (cPWR_UsePowerDownMode)
  /*---------------------------------------------------------------------------*/
#if (cPWR_SleepMode==0)
  return Res;
  
  /*---------------------------------------------------------------------------*/
#elif (cPWR_SleepMode==1)
  /* radio in autodoze mode by default. mcu in wait mode */
  PWRLib_MCU_WakeupReason.AllBits = 0;
  Cpu_SetOperationMode(DOM_WAIT,NULL,NULL);
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
PWR_CheckForAndEnterNewPowerState_t NewPowerState
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
    Cpu_SetClockConfiguration(gClockConfig_FEI_24_c);
    /* configure Radio in sleep mode */
    (void)MKW01Drv_RadioSleepReq();
    // disable all wake up sources
    LLWU_PE1 = 0;
    LLWU_PE2 = 0;
    LLWU_PE3 = 0;
    LLWU_PE4 = 0;
    LLWU_ME = 0;
    /* configure MCU in VLLS1 mode */
    PWRLib_MCU_Enter_VLLS1();
    /* Never returns */
    for(;;){}
    
  }
  else if( NewPowerState == PWR_Reset)
  {
    /* Never returns */
    PWRLib_Reset();
  }
  
  else if( NewPowerState == PWR_DeepSleep )
  {
    if(PWR_CheckIfDeviceCanGoToSleep() && PWR_DeepSleepAllowed())
    {
      ReturnValue = PWR_HandleDeepSleep();
    }
  } 
  else if( NewPowerState == PWR_Sleep )
  {
    if(PWR_CheckIfDeviceCanGoToSleep() && PWR_SleepAllowed())
    {
      ReturnValue = PWR_HandleSleep();
    }
  }
  else
  {
    /* ReturnValue = FALSE; */
  }
  /* Clear wakeup reason */
  
#else
  /* To remove warning for variabels in functioncall */
  (void)NewPowerState;
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
  OSA_EXT_InterruptDisable();
  (void)PWR_CheckForAndEnterNewPowerState(PWR_OFF);
  OSA_EXT_InterruptEnable();
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
    ReturnValue = PWR_CheckForAndEnterNewPowerState( PWR_OFF);
  }
  OSA_EXT_InterruptDisable();
  
#if (cPWR_UsePowerModuleStandAlone == 0)
  PWRLib_SetCurrentZigbeeStackPowerState(StackPS_DeepSleep);
#endif
  if (
      TMR_AreAllTimersOff()
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
    
    ReturnValue = PWR_CheckForAndEnterNewPowerState (PWR_DeepSleep);
  }
  else /*timers are running*/
  { 	 
    ReturnValue = PWR_CheckForAndEnterNewPowerState (PWR_Sleep);
  }
  OSA_EXT_InterruptEnable();
  
#if (gTMR_EnableLowPowerTimers_d)
  if(unlockTMRThread)
  {
    TMR_MakeTMRThreadReady();
  }
  
#endif    
  return ReturnValue;
}
