/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PWRLib.c
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
#include "PWRLib.h"
#include "PWR_Configuration.h"
#include "TimersManager.h"
#include "Keyboard.h"
#include "MCR20Drv.h"
#include "MCR20Reg.h"

#include "TMR_Adapter.h"


#include "fsl_os_abstraction.h"
#include "fsl_lptmr_hal.h"
#include "fsl_lptmr_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_llwu_hal.h"
#include "fsl_smc_hal.h"

#if (cPWR_UsePowerModuleStandAlone == 0)
#include "MacInterface.h"
#endif
/*****************************************************************************
 *                               PRIVATE VARIABLES                           *
 *---------------------------------------------------------------------------*
 * Add to this section all the variables and constants that have local       *
 * (file) scope.                                                             *
 * Each of this declarations shall be preceded by the 'static' keyword.      *
 * These variables / constants cannot be accessed outside this module.       *
 *---------------------------------------------------------------------------*
 *****************************************************************************/

/* LPTMR/RTC variables */
   
#if (cPWR_UsePowerDownMode==1)
static uint32_t mPWRLib_RTIElapsedTicks;
#endif /* #if (cPWR_UsePowerDownMode==1) */


/* For LVD function */ 

#if (cPWR_LVD_Enable == 2)
tmrTimerID_t               PWRLib_LVD_PollIntervalTmrID;
PWRLib_LVD_VoltageLevel_t  PWRLib_LVD_SavedLevel;
#endif  /* #if (cPWR_LVD_Enable == 2) */


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

/* Zigbee STACK status */ 
PWRLib_StackPS_t PWRLib_StackPS;
volatile PWRLib_WakeupReason_t PWRLib_MCU_WakeupReason;

#if (cPWR_UsePowerDownMode==1)

/*****************************************************************************
 *                           PRIVATE FUNCTIONS PROTOTYPES                    *
 *---------------------------------------------------------------------------*
 * Add to this section all the functions prototypes that have local (file)   *
 * scope.                                                                    *
 * These functions cannot be accessed outside this module.                   *
 * These declarations shall be preceded by the 'static' keyword.             *
 *---------------------------------------------------------------------------*
 *****************************************************************************/


/*****************************************************************************
 *                                PRIVATE FUNCTIONS                          *
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
 * Name: PWRLib_Radio_Enter_Doze
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_Doze
(
void
)
{
  uint8_t phyCtrl1Reg, irqSts1Reg, pwrModesReg;
  OSA_EnterCritical(kCriticalDisableInt);
  pwrModesReg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES);
  /* disable autodoze mode. sets PMC in low-power mode */
  pwrModesReg &= (uint8_t) ~( cPWR_MODES_AUTODOZE | cPWR_MODES_PMC_MODE );
  /* check if 32 MHz crystal oscillator is enabled (current state is hibernate mode) */
  if( (pwrModesReg & cPWR_MODES_XTALEN ) != cPWR_MODES_XTALEN )
  {
    /* enable 32 MHz crystal oscillator */
    pwrModesReg |= (uint8_t) cPWR_MODES_XTALEN;
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
    /* wait for crystal oscillator to complet its warmup */
    while( ( MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES) & cPWR_MODES_XTAL_READY ) != cPWR_MODES_XTAL_READY);
    /* wait for radio wakeup from hibernate interrupt */
    while( ( MCR20Drv_DirectAccessSPIRead( (uint8_t) IRQSTS2) & (cIRQSTS2_WAKE_IRQ | cIRQSTS2_TMRSTATUS) ) != (cIRQSTS2_WAKE_IRQ | cIRQSTS2_TMRSTATUS) );
    MCR20Drv_DirectAccessSPIWrite((uint8_t) IRQSTS2, (uint8_t) (cIRQSTS2_WAKE_IRQ));
  }
  else
  {
    /* checks if packet processor is in idle state. otherwise abort any ongoing sequence */
    phyCtrl1Reg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PHY_CTRL1 );
    if( (phyCtrl1Reg & cPHY_CTRL1_XCVSEQ) != 0x00 )
    {
      /* abort any ongoing sequence */
      /* make sure that we abort in HW only if the sequence was actually started (tmr triggered) */
      if( ( 0 != ( MCR20Drv_DirectAccessSPIRead( (uint8_t) PHY_CTRL1) & cPHY_CTRL1_XCVSEQ ) ) && ((MCR20Drv_DirectAccessSPIRead(SEQ_STATE)&0x1F) != 0))
      {
        phyCtrl1Reg &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
        MCR20Drv_DirectAccessSPIWrite(PHY_CTRL1, phyCtrl1Reg);
        while ((MCR20Drv_DirectAccessSPIRead(SEQ_STATE) & 0x1F) != 0);
      }
      /* clear sequence-end interrupt */ 
      irqSts1Reg = MCR20Drv_DirectAccessSPIRead( (uint8_t) IRQSTS1);
      irqSts1Reg |= (uint8_t) cIRQSTS1_SEQIRQ;
      MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS1, irqSts1Reg);
    }
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
  }
  OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
 * Name: PWRLib_Radio_Enter_AutoDoze
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_AutoDoze
(
void
)
{
  uint8_t pwrModesReg;
  OSA_EnterCritical(kCriticalDisableInt);
  pwrModesReg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES);
  /* enable autodoze mode. */
  pwrModesReg |= (uint8_t) cPWR_MODES_AUTODOZE;
  /* check if 32 MHz crystal oscillator is enabled (current state is hibernate mode) */
  if( (pwrModesReg & cPWR_MODES_XTALEN ) != cPWR_MODES_XTALEN )
  {
    /* enable 32 MHz crystal oscillator */
    pwrModesReg |= (uint8_t) cPWR_MODES_XTALEN;
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
    /* wait for crystal oscillator to complet its warmup */
    while( ( MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES) & cPWR_MODES_XTAL_READY ) != cPWR_MODES_XTAL_READY);
    /* wait for radio wakeup from hibernate interrupt */
    while( ( MCR20Drv_DirectAccessSPIRead( (uint8_t) IRQSTS2) & (cIRQSTS2_WAKE_IRQ | cIRQSTS2_TMRSTATUS) ) != (cIRQSTS2_WAKE_IRQ | cIRQSTS2_TMRSTATUS) );
    MCR20Drv_DirectAccessSPIWrite((uint8_t) IRQSTS2, (uint8_t) (cIRQSTS2_WAKE_IRQ));
  }
  else
  {
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
  }
  OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
 * Name: PWRLib_Radio_Enter_Idle
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_Idle
(
void
)
{
  uint8_t phyCtrl1Reg, irqSts1Reg, pwrModesReg;
  OSA_EnterCritical(kCriticalDisableInt);
  pwrModesReg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES);
  /* disable autodoze mode. sets PMC in high-power mode */
  pwrModesReg &= (uint8_t) ~( cPWR_MODES_AUTODOZE );
  pwrModesReg |= (uint8_t) cPWR_MODES_PMC_MODE;
  /* check if 32 MHz crystal oscillator is enabled (current state is hibernate mode) */
  if( (pwrModesReg & cPWR_MODES_XTALEN ) != cPWR_MODES_XTALEN )
  {
    /* enable 32 MHz crystal oscillator */
    pwrModesReg |= (uint8_t) cPWR_MODES_XTALEN;
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
    /* wait for crystal oscillator to complet its warmup */
    while( ( MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES) & cPWR_MODES_XTAL_READY ) != cPWR_MODES_XTAL_READY);
    /* wait for radio wakeup from hibernate interrupt */
    while( ( MCR20Drv_DirectAccessSPIRead( (uint8_t) IRQSTS2) & (cIRQSTS2_WAKE_IRQ | cIRQSTS2_TMRSTATUS) ) != (cIRQSTS2_WAKE_IRQ | cIRQSTS2_TMRSTATUS) );
    MCR20Drv_DirectAccessSPIWrite((uint8_t) IRQSTS2, (uint8_t) (cIRQSTS2_WAKE_IRQ));
  }
  else
  {
    /* checks if packet processor is in idle state. otherwise abort any ongoing sequence */
    phyCtrl1Reg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PHY_CTRL1 );
    if( (phyCtrl1Reg & cPHY_CTRL1_XCVSEQ) != 0x00 )
    {
      /* abort any ongoing sequence */
      /* make sure that we abort in HW only if the sequence was actually started (tmr triggered) */
      if( ( 0 != ( MCR20Drv_DirectAccessSPIRead( (uint8_t) PHY_CTRL1) & cPHY_CTRL1_XCVSEQ ) ) && ((MCR20Drv_DirectAccessSPIRead(SEQ_STATE)&0x1F) != 0))
      {
        phyCtrl1Reg &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
        MCR20Drv_DirectAccessSPIWrite(PHY_CTRL1, phyCtrl1Reg);
        while ((MCR20Drv_DirectAccessSPIRead(SEQ_STATE) & 0x1F) != 0);
      }
      /* clear sequence-end interrupt */ 
      irqSts1Reg = MCR20Drv_DirectAccessSPIRead( (uint8_t) IRQSTS1);
      irqSts1Reg |= (uint8_t) cIRQSTS1_SEQIRQ;
      MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS1, irqSts1Reg);
    }
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
  }
  OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
 * Name: PWRLib_Radio_Enter_Hibernate
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Radio_Enter_Hibernate
(
void
)
{
  uint8_t phyCtrl1Reg, irqSts1Reg, pwrModesReg;
  OSA_EnterCritical(kCriticalDisableInt);
  /* checks if packet processor is in idle state. otherwise abort any ongoing sequence */
  phyCtrl1Reg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PHY_CTRL1 );
  if( (phyCtrl1Reg & cPHY_CTRL1_XCVSEQ) != 0x00 )
  {
    /* abort any ongoing sequence */
    /* make sure that we abort in HW only if the sequence was actually started (tmr triggered) */
    if( ( 0 != ( MCR20Drv_DirectAccessSPIRead( (uint8_t) PHY_CTRL1) & cPHY_CTRL1_XCVSEQ ) ) && ((MCR20Drv_DirectAccessSPIRead(SEQ_STATE)&0x1F) != 0))
    {
      phyCtrl1Reg &= (uint8_t) ~(cPHY_CTRL1_XCVSEQ);
      MCR20Drv_DirectAccessSPIWrite(PHY_CTRL1, phyCtrl1Reg);
      while ((MCR20Drv_DirectAccessSPIRead(SEQ_STATE) & 0x1F) != 0);
    }
    /* clear sequence-end interrupt */ 
    irqSts1Reg = MCR20Drv_DirectAccessSPIRead( (uint8_t) IRQSTS1);
    irqSts1Reg |= (uint8_t) cIRQSTS1_SEQIRQ;
    MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS1, irqSts1Reg);
  }
  
  pwrModesReg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES);
  /* disable autodoze mode. disable 32 MHz crystal oscillator. sets PMC in low-power mode */
  pwrModesReg &= (uint8_t) ~( cPWR_MODES_AUTODOZE | cPWR_MODES_XTALEN | cPWR_MODES_PMC_MODE );
  
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
  
  //  {
  //    uint8_t tmpReg;
  //    tmpReg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES);
  //    while( cPWR_MODES_XTAL_READY == ( tmpReg & cPWR_MODES_XTAL_READY ) )
  //    {
  //      MCR20Drv_DirectAccessSPIWrite( (uint8_t) PWR_MODES, pwrModesReg);
  //      tmpReg = MCR20Drv_DirectAccessSPIRead( (uint8_t) PWR_MODES);
  //    }
  //  }
  OSA_ExitCritical(kCriticalDisableInt);
}



/*---------------------------------------------------------------------------
 * Name: PWRLib_LLWU_UpdateWakeupReason
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_LLWU_UpdateWakeupReason
(
void
)
{
  uint32_t  i;

  for(i=0; i<15; i++)
  {
      if( ((1<<i) & gPWRLib_LLWU_KeyboardMask_c) &&
          LLWU_HAL_GetExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)i) )
      {
          PWRLib_MCU_WakeupReason.Bits.FromKeyBoard = 1;
          break;
      }
  }

  if( LLWU_HAL_GetInternalModuleWakeupFlag(LLWU_BASE, gPWRLib_LLWU_WakeupModule_LPTMR_c) )
  {
    PWRLib_MCU_WakeupReason.Bits.FromLPTMR = 1;
    PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout = 1;
  }

  if( LLWU_HAL_GetInternalModuleWakeupFlag(LLWU_BASE, gPWRLib_LLWU_WakeupModule_RTC_Alarm_c) )
  {
    PWRLib_MCU_WakeupReason.Bits.FromRTC = 1;
    PWRLib_MCU_WakeupReason.Bits.DeepSleepTimeout = 1;
  }
}





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
)
{
    uint32_t baseAddr = g_lptmrBaseAddr[gLptmrInstance_c];

    OSA_EnterCritical(kCriticalDisableInt);
    LPTMR_HAL_Disable(baseAddr);
    /* Set compare value */
    LPTMR_HAL_SetCompareValue(baseAddr, Ticks);
    /* Use specified tick count */
    mPWRLib_RTIElapsedTicks = 0;
    /* Configure prescaler, bypass prescaler and clck source */
    if( ClkMode == cLPTMR_PRS_00001ms )
    {
        /* Disable LPTMR prescaller: clock divider = 1 */
        LPTMR_HAL_SetPrescalerCmd(baseAddr, 0);
    }
    else
    {
        /* Enable LPTMR prescaler: clock divider is between 2 and 65536 */
        LPTMR_HAL_SetPrescalerCmd(baseAddr, 1);
        LPTMR_HAL_SetPrescalerValueMode(baseAddr, (lptmr_prescaler_value_t)ClkMode);
    }
    LPTMR_HAL_SetPrescalerClockSourceMode(baseAddr, cPWR_LPTMRClockSource);
    /* Start counting */
    LPTMR_HAL_Enable(baseAddr);
    OSA_ExitCritical(kCriticalDisableInt);
}

/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_ClockCheck
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint32_t PWRLib_LPTMR_ClockCheck
(
void
)
{
    uint32_t baseAddr = g_lptmrBaseAddr[gLptmrInstance_c];
    
    OSA_EnterCritical(kCriticalDisableInt);
    /* LPTMR is still running */
    if( LPTMR_HAL_IsEnabled(baseAddr) )
    {
        mPWRLib_RTIElapsedTicks = LPTMR_HAL_GetCounterValue(baseAddr);
        /* timer compare flag is set */
        if( LPTMR_HAL_IsIntPending(baseAddr) )
        {
            uint32_t compareReg;
            compareReg = LPTMR_HAL_GetCompareValue(baseAddr);
            if(mPWRLib_RTIElapsedTicks < compareReg )
            {
                mPWRLib_RTIElapsedTicks += 0x10000;
            }
        }
    }
    OSA_ExitCritical(kCriticalDisableInt);
    return mPWRLib_RTIElapsedTicks;
}



/*---------------------------------------------------------------------------
 * Name: PWRLib_LPTMR_ClockStop
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_LPTMR_ClockStop
(
void
)
{
    uint32_t baseAddr = g_lptmrBaseAddr[gLptmrInstance_c];

    OSA_EnterCritical(kCriticalDisableInt);
    /* LPTMR is still running */
    if( LPTMR_HAL_IsEnabled(baseAddr) )
    {
        mPWRLib_RTIElapsedTicks = LPTMR_HAL_GetCounterValue(baseAddr);
        /* timer compare flag is set */
        if( LPTMR_HAL_IsIntPending(baseAddr) )
        {
            uint32_t compareReg;
            compareReg = LPTMR_HAL_GetCompareValue(baseAddr);
            if(mPWRLib_RTIElapsedTicks < compareReg )
            {
                mPWRLib_RTIElapsedTicks += 0x10000;
            }
        }
    }
    /* Stop LPTMR */
    LPTMR_HAL_Disable(baseAddr);
    OSA_ExitCritical(kCriticalDisableInt);
}


#if (cPWR_UsePowerModuleStandAlone == 0)

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
)
{
  return Mac_GetState();
}

#endif /* (cPWR_UsePowerModuleStandAlone == 0) */

/*---------------------------------------------------------------------------
 * Name: PWRLib_LLWU_Isr
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/

void PWRLib_LLWU_Isr
(
void
)
{
    /* Clear external pins wakeup interrupts */
    LLWU_F1 = LLWU_F1; 
    LLWU_F2 = LLWU_F2; 
    
    /* LPTMR is wakeup source */
    if( LLWU_HAL_GetInternalModuleWakeupFlag(LLWU_BASE, gPWRLib_LLWU_WakeupModule_LPTMR_c) )
    {
        uint32_t baseAddr = g_lptmrBaseAddr[gLptmrInstance_c];
        /* Clear LPTMR interrupt */
        LPTMR_HAL_ClearIntFlag(baseAddr);
    }
    /* RTC alarm is wakeup source */
    if( LLWU_HAL_GetInternalModuleWakeupFlag(LLWU_BASE, gPWRLib_LLWU_WakeupModule_RTC_Alarm_c) )
    {
        uint32_t rtcBaseAddr = g_rtcBaseAddr[gTmrRtcInstance_c];
        /* Clear alarm interrupt flag */
        RTC_HAL_SetSecsReg(rtcBaseAddr, RTC_HAL_GetSecsReg(rtcBaseAddr) );
    }
}

#endif /* #if (cPWR_UsePowerDownMode==1) */




/*---------------------------------------------------------------------------
* Name: PWRLib_LVD_CollectLevel
* Description: -
* Parameters: -
* Return: -
*---------------------------------------------------------------------------*/
PWRLib_LVD_VoltageLevel_t PWRLib_LVD_CollectLevel
(
void
)
{
#if ((cPWR_LVD_Enable == 1) || (cPWR_LVD_Enable == 2))
  
  /* Check low detect voltage 1.6V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(0);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(0);
  PMC_LVDSC1 = PMC_LVDSC1_LVDACK_MASK;
  if(PMC_LVDSC1 & PMC_LVDSC1_LVDF_MASK)
  {
    /* Low detect voltage reached */
    PMC_LVDSC1 = PMC_LVDSC1_LVDACK_MASK;
    return(PWR_LEVEL_CRITICAL);
  }
  
  /* Check low trip voltage 1.8V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(0);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(0);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_1_8V);
  }
  
  /* Check low trip voltage 1.9V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(0);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(1);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_1_9V);
  }
  /* Check low trip voltage 2.0V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(0);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(2);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_2_0V);
  }
  
  /* Check low trip voltage 2.1V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(0);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(3);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_2_1V);
  }
  
  /* Check low detect voltage (high range) 2.56V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(1); /* Set high trip voltage and clear warning flag */
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(0);
  PMC_LVDSC1 |= PMC_LVDSC1_LVDACK_MASK;
  if(PMC_LVDSC1 & PMC_LVDSC1_LVDF_MASK)
  {
    /* Low detect voltage reached */
    PMC_LVDSC1 = PMC_LVDSC1_LVDACK_MASK; /* Set low trip voltage and clear warning flag */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_2_56V);
  }
  
  /* Check high trip voltage 2.7V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(1);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(0);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_2_7V);
  }
  
  /* Check high trip voltage 2.8V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(1);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(1);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_2_8V);
  }
  
  /* Check high trip voltage 2.9V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(1);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(2);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_2_9V);
  }
  
  /* Check high trip voltage 3.0V */
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(1);
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(3);
  PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
  if(PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK)
  {
    /* Low trip voltage reached */
    PMC_LVDSC2 = PMC_LVDSC2_LVWACK_MASK; /* Clear flag (and set low trip voltage) */
    PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
    return(PWR_BELOW_LEVEL_3_0V);
  }
  
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(0);
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(0); /* Set low trip voltage */
#endif  /* #if ((cPWR_LVD_Enable == 1) || (cPWR_LVD_Enable == 2)) */
  
  /*--- Voltage level is okay > 3.0V */
  return(PWR_ABOVE_LEVEL_3_0V);
}

/******************************************************************************
 * Name: PWRLib_LVD_PollIntervalCallback
 * Description:
 *
 * Parameter(s): -
 * Return: -
 ******************************************************************************/
#if (cPWR_LVD_Enable == 2)
static void PWRLib_LVD_PollIntervalCallback
(
void* param
)
{
  (void)param;
  PWRLib_LVD_SavedLevel = PWRLib_LVD_CollectLevel();
}
#endif



/*---------------------------------------------------------------------------
 * Name: PWRLib_GetSystemResetStatus
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
uint16_t PWRLib_GetSystemResetStatus
(
  void
)
{
  uint16_t resetStatus = 0;
  resetStatus = (uint16_t) (RCM_SRS0);
  resetStatus |= (uint16_t)(RCM_SRS1 << 8);
  return resetStatus;
}

/*---------------------------------------------------------------------------
 * Name: PWRLib_Init
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Init
(
void
)
{
#if (cPWR_UsePowerDownMode == 1)
    smc_power_mode_protection_config_t smcProtConfig = {
        .vlpProt  = true,
        .vllsProt = true,
#if FSL_FEATURE_SMC_HAS_LOW_LEAKAGE_STOP_MODE
        .llsProt  = true,
#endif
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE
        .hsrunProt = true
#endif
    };
    
    /* allow entering specific modes */
    SMC_HAL_SetProtection(SMC_BASE, &smcProtConfig);
    
  /* enable clock to LLWU module */  
#if ( (cPWR_DeepSleepMode != 0) && (cPWR_DeepSleepMode != 13) )  
  PWRLib_LLWU_UpdateWakeupReason();
#endif

#if ( (cPWR_DeepSleepMode == 4) || (cPWR_DeepSleepMode == 7) || (cPWR_DeepSleepMode == 11) )
  TMR_RTCInit();
  LLWU_HAL_SetInternalModuleCmd(LLWU_BASE, gPWRLib_LLWU_WakeupModule_RTC_Alarm_c, true);
#endif
  
#if ( (cPWR_DeepSleepMode == 2) || (cPWR_DeepSleepMode == 3) || (cPWR_DeepSleepMode == 5) || (cPWR_DeepSleepMode == 6) || (cPWR_DeepSleepMode == 8) || (cPWR_DeepSleepMode == 9) || (cPWR_DeepSleepMode == 10) || (cPWR_DeepSleepMode == 12) || (cPWR_DeepSleepMode == 14) || (cPWR_DeepSleepMode == 15) )  
  /* configure NVIC for LPTMR Isr */
  LPTMR_Init(NULL);
  /* enable LPTMR as wakeup source for LLWU module */
  LLWU_HAL_SetInternalModuleCmd(LLWU_BASE, gPWRLib_LLWU_WakeupModule_LPTMR_c, true);
#endif

#if ( (cPWR_DeepSleepMode != 0) && (cPWR_DeepSleepMode != 2) && (cPWR_DeepSleepMode != 3) && (cPWR_DeepSleepMode != 4) )
  LLWU_HAL_SetExternalInputPinMode(LLWU_BASE, kLlwuExternalPinChangeDetect, gPWRLib_LLWU_WakeupPin_PTA4_c);
  LLWU_HAL_SetExternalInputPinMode(LLWU_BASE, kLlwuExternalPinChangeDetect, gPWRLib_LLWU_WakeupPin_PTC6_c);
#endif

#if ( (cPWR_DeepSleepMode != 0) && (cPWR_DeepSleepMode != 13) )
  /* install LLWU Isr and validate it in NVIC */
  OSA_InstallIntHandler (LLW_IRQn, PWRLib_LLWU_Isr);
  NVIC_EnableIRQ(LLW_IRQn);
#endif
#endif /* #if (cPWR_UsePowerDownMode==1) */
  
  /* LVD_Init TODO */
#if (cPWR_LVD_Enable == 0)
  PMC_LVDSC1 &= (uint8_t) ~( PMC_LVDSC1_LVDIE_MASK  | PMC_LVDSC1_LVDRE_MASK);
  PMC_LVDSC2 &= (uint8_t) ~( PMC_LVDSC2_LVWIE_MASK );
#elif ((cPWR_LVD_Enable == 1) || (cPWR_LVD_Enable == 2))
  PMC_LVDSC1 &= (uint8_t) ~( PMC_LVDSC1_LVDIE_MASK | PMC_LVDSC1_LVDRE_MASK);
  PMC_LVDSC2 &= (uint8_t) ~( PMC_LVDSC2_LVWIE_MASK );
#elif (cPWR_LVD_Enable==3)
  PMC_LVDSC1 = (PMC_LVDSC1 | (uint8_t)PMC_LVDSC1_LVDRE_MASK) & (uint8_t)(~PMC_LVDSC1_LVDIE_MASK );
  PMC_LVDSC2 &= (uint8_t) ~( PMC_LVDSC2_LVWIE_MASK );
#endif /* #if (cPWR_LVD_Enable) */
  
  
#if (cPWR_LVD_Enable == 2)
#if ((cPWR_LVD_Ticks == 0) || (cPWR_LVD_Ticks > 71582))
#error  "*** ERROR: cPWR_LVD_Ticks invalid value"
#endif 
  
  PWRLib_LVD_SavedLevel = PWRLib_LVD_CollectLevel(); 
  /* Allocate a platform timer */
  PWRLib_LVD_PollIntervalTmrID = TMR_AllocateTimer();   
  if(gTmrInvalidTimerID_c != PWRLib_LVD_PollIntervalTmrID)
  { 
    /* start the timer */
    TMR_StartLowPowerTimer(PWRLib_LVD_PollIntervalTmrID, gTmrIntervalTimer_c,TmrMinutes(cPWR_LVD_Ticks) , PWRLib_LVD_PollIntervalCallback, NULL); 
  }
#endif  /* #if (cPWR_LVD_Enable==2) */
  
}

/*---------------------------------------------------------------------------
 * Name: PWRLib_Reset
 * Description: -
 * Parameters: -
 * Return: -
 *---------------------------------------------------------------------------*/
void PWRLib_Reset
(
  void
)
{
  NVIC_SystemReset();
  while(1);
}
