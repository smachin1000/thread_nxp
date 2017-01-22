/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyTime.c
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


/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "MCR20Drv.h"
#include "MCR20Reg.h"
#include "Phy.h"

#include "FunctionLib.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define gPhyTimeMinSetupTime_c (10) /* symbols */

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
void (*gpfPhyTimeNotify)(void) = NULL;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static phyTimeEvent_t  mPhyTimers[gMaxPhyTimers_c];
static phyTimeEvent_t *pNextEvent;
volatile uint32_t      mPhySeqTimeout;
volatile uint64_t      gPhyTimerOverflow;

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void PhyTime_OverflowCB( uint32_t param );
static phyTimeEvent_t* PhyTime_GetNextEvent( void );

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  Sets the start time of a sequence
*
* \param[in]  startTime  the start time for a sequence
*
********************************************************************************** */
void PhyTimeSetEventTrigger
(
  uint32_t startTime
)
{
  uint8_t phyReg, phyCtrl3Reg;

  OSA_EnterCritical(kCriticalDisableInt);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
  phyReg |= cPHY_CTRL1_TMRTRIGEN;    // enable autosequence start by TC2 match
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);

  phyCtrl3Reg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
  phyCtrl3Reg &= ~(cPHY_CTRL3_TMR2CMP_EN);// disable TMR2 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  MCR20Drv_DirectAccessSPIMultiByteWrite( (uint8_t) T2PRIMECMP_LSB, (uint8_t *) &startTime, 2);

  phyReg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);
  phyReg &= 0xF0;                     // do not change other IRQs status
  phyReg &= ~(cIRQSTS3_TMR2MSK);      // unmask TMR2 interrupt
  phyReg |= (cIRQSTS3_TMR2IRQ);       // aknowledge TMR2 IRQ
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, phyReg);

  // TC2PRIME_EN must be enabled in PHY_CTRL4 register
  phyCtrl3Reg |= cPHY_CTRL3_TMR2CMP_EN;   // enable TMR2 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  Disable the time trigger for a sequence.
*
* \remarks The sequence will start asap
*
********************************************************************************** */
void PhyTimeDisableEventTrigger
(
  void
)
{
  uint8_t phyReg;

  OSA_EnterCritical(kCriticalDisableInt);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL1);
  phyReg &= ~(cPHY_CTRL1_TMRTRIGEN); // disable autosequence start by TC2 match
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL1, phyReg);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
  phyReg &= ~(cPHY_CTRL3_TMR2CMP_EN);// disable TMR2 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyReg);

  phyReg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);
  phyReg &= 0xF0;                    // do not change other IRQs status
  phyReg |= (cIRQSTS3_TMR2MSK);      // mask TMR2 interrupt
  phyReg |= (cIRQSTS3_TMR2IRQ);      // aknowledge TMR2 IRQ
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, phyReg);

  OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  Sets the timeout value for a sequence
*
* \param[in]  pEndTime the absolute time when a sequence should terminate
*
* \remarks If the sequence does not finish until the timeout, it will be aborted
*
********************************************************************************** */
void PhyTimeSetEventTimeout
(
  uint32_t *pEndTime
)
{
  uint8_t phyReg, phyCtrl3Reg;

#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pEndTime)
  {
    return;
  }
#endif // PHY_PARAMETERS_VALIDATION

  OSA_EnterCritical(kCriticalDisableInt);

  phyCtrl3Reg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
  phyCtrl3Reg &= ~(cPHY_CTRL3_TMR3CMP_EN);// disable TMR3 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL4);
  phyReg |= cPHY_CTRL4_TC3TMOUT;     // enable autosequence stop by TC3 match
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL4, phyReg);

  mPhySeqTimeout = *pEndTime & 0x00FFFFFF;
  MCR20Drv_DirectAccessSPIMultiByteWrite( (uint8_t) T3CMP_LSB, (uint8_t *) pEndTime, 3);

  phyReg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);
  phyReg &= 0xF0;                     // do not change IRQ status
//  phyReg &= ~(cIRQSTS3_TMR3MSK);      // unmask TMR3 interrupt
  phyReg |= (cIRQSTS3_TMR3IRQ);       // aknowledge TMR3 IRQ
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, phyReg);

  phyCtrl3Reg |= cPHY_CTRL3_TMR3CMP_EN;   // enable TMR3 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  Return the timeout value for the current sequence
*
* \return  uint32_t the timeout value
*
********************************************************************************** */
uint32_t PhyTimeGetEventTimeout( void )
{
    return mPhySeqTimeout;
}

/*! *********************************************************************************
* \brief  Disables the sequence timeout
*
********************************************************************************** */
void PhyTimeDisableEventTimeout
(
  void
)
{
  uint8_t phyReg;

  OSA_EnterCritical(kCriticalDisableInt);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL4);
  phyReg &= ~(cPHY_CTRL4_TC3TMOUT);  // disable autosequence stop by TC3 match
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL4, phyReg);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
  phyReg &= ~(cPHY_CTRL3_TMR3CMP_EN);// disable TMR3 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyReg);

  phyReg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);
  phyReg &= 0xF0;                     // do not change IRQ status
  phyReg |= cIRQSTS3_TMR3IRQ;         // aknowledge TMR3 IRQ
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, phyReg);

  OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  Reads the absolute clock from the radio
*
* \param[out]  pRetClk pointer to a location where the current clock will be stored
*
********************************************************************************** */
void PhyTimeReadClock
(
  uint32_t *pRetClk
)
{
#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pRetClk)
  {
    return;
  }
#endif // PHY_PARAMETERS_VALIDATION

  OSA_EnterCritical(kCriticalDisableInt);

  MCR20Drv_DirectAccessSPIMultiByteRead( (uint8_t) EVENT_TMR_LSB, (uint8_t *) pRetClk, 3);
  *(((uint8_t *)pRetClk) + 3) = 0;

  OSA_ExitCritical(kCriticalDisableInt);

}

/*! *********************************************************************************
* \brief  Initialize the Event Timer
*
* \param[in]  pAbsTime  pointer to the location where the new time is stored
*
********************************************************************************** */
void PhyTimeInitEventTimer
(
  uint32_t *pAbsTime
)
{
  uint8_t phyCtrl4Reg;

#ifdef PHY_PARAMETERS_VALIDATION
  if(NULL == pAbsTime)
  {
    return;
  }
#endif // PHY_PARAMETERS_VALIDATION

  OSA_EnterCritical(kCriticalDisableInt);

  phyCtrl4Reg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL4);
  phyCtrl4Reg |= cPHY_CTRL4_TMRLOAD; // self clearing bit

  MCR20Drv_DirectAccessSPIMultiByteWrite( (uint8_t) T1CMP_LSB, (uint8_t *) pAbsTime, 3);
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL4, phyCtrl4Reg);

  OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  Set TMR1 timeout value
*
* \param[in]  pWaitTimeout the timeout value
*
********************************************************************************** */
void PhyTimeSetWaitTimeout
(
  uint32_t *pWaitTimeout
)
{
  uint8_t phyCtrl3Reg, irqSts3Reg;

  OSA_EnterCritical(kCriticalDisableInt);

  phyCtrl3Reg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
  phyCtrl3Reg &= ~(cPHY_CTRL3_TMR1CMP_EN);// disable TMR1 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  MCR20Drv_DirectAccessSPIMultiByteWrite( (uint8_t) T1CMP_LSB, (uint8_t *) pWaitTimeout, 3);

  irqSts3Reg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);
  irqSts3Reg &= ~(cIRQSTS3_TMR1MSK);      // unmask TMR1 interrupt
  irqSts3Reg &= 0xF0;                     // do not change other IRQs status
  irqSts3Reg |= (cIRQSTS3_TMR1IRQ);       // aknowledge TMR1 IRQ
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, irqSts3Reg);

  phyCtrl3Reg |= cPHY_CTRL3_TMR1CMP_EN;   // enable TMR1 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  OSA_ExitCritical(kCriticalDisableInt);

}

/*! *********************************************************************************
* \brief  Disable the TMR1 timeout
*
********************************************************************************** */
void PhyTimeDisableWaitTimeout
(
  void
)
{
  uint8_t phyReg;

  OSA_EnterCritical(kCriticalDisableInt);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
  phyReg &= ~(cPHY_CTRL3_TMR1CMP_EN);// disable TMR1 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyReg);

  phyReg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);
  phyReg &= 0xF0;                     // do not change IRQ status
  phyReg |= cIRQSTS3_TMR1IRQ;         // aknowledge TMR1 IRQ
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, phyReg);

  OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  Set TMR4 timeout value
*
* \param[in]  pWakeUpTime  absolute time
*
********************************************************************************** */
void PhyTimeSetWakeUpTime
(
  uint32_t *pWakeUpTime
)
{
  uint8_t phyCtrl3Reg, irqSts3Reg;

  OSA_EnterCritical(kCriticalDisableInt);

  phyCtrl3Reg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
//  phyCtrl3Reg &= ~(cPHY_CTRL3_TMR4CMP_EN);// disable TMR4 compare
//  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  MCR20Drv_DirectAccessSPIMultiByteWrite( (uint8_t) T4CMP_LSB, (uint8_t *) pWakeUpTime, 3);

  irqSts3Reg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);
  irqSts3Reg &= ~(cIRQSTS3_TMR4MSK);      // unmask TMR4 interrupt
  irqSts3Reg &= 0xF0;                     // do not change other IRQs status
  irqSts3Reg |= (cIRQSTS3_TMR4IRQ);       // aknowledge TMR4 IRQ
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, irqSts3Reg);

  phyCtrl3Reg |= cPHY_CTRL3_TMR4CMP_EN;   // enable TMR4 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyCtrl3Reg);

  OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  Check if TMR4 IRQ occured, and aknowledge it
*
* \return  TRUE if TMR4 IRQ occured
*
********************************************************************************** */
bool_t PhyTimeIsWakeUpTimeExpired
(
  void
)
{
  bool_t wakeUpIrq = FALSE;
  uint8_t phyReg;

  OSA_EnterCritical(kCriticalDisableInt);

  phyReg = MCR20Drv_DirectAccessSPIRead(PHY_CTRL3);
  phyReg &= ~(cPHY_CTRL3_TMR4CMP_EN);// disable TMR4 compare
  MCR20Drv_DirectAccessSPIWrite( (uint8_t) PHY_CTRL3, phyReg);

  phyReg = MCR20Drv_DirectAccessSPIRead(IRQSTS3);

  if( (phyReg & cIRQSTS3_TMR4IRQ) == cIRQSTS3_TMR4IRQ )
  {
    wakeUpIrq = TRUE;
  }

  phyReg &= ~(cIRQSTS3_TMR4MSK);      // unmask TMR4 interrupt
  phyReg &= 0xF0;                     // do not change other IRQs status
  phyReg |= (cIRQSTS3_TMR4IRQ);       // aknowledge TMR2 IRQ

  MCR20Drv_DirectAccessSPIWrite( (uint8_t) IRQSTS3, phyReg);

  OSA_ExitCritical(kCriticalDisableInt);

  return wakeUpIrq;
}


/*! *********************************************************************************
* \brief  Initialize the PHY Timer module
*
* \return  phyTimeStatus_t
*
********************************************************************************** */
phyTimeStatus_t PhyTime_TimerInit( void (*cb)(void) )
{
    if( gpfPhyTimeNotify )
        return gPhyTimeError_c;

    gpfPhyTimeNotify = cb;
    gPhyTimerOverflow = 0;
    FLib_MemSet( mPhyTimers, 0, sizeof(mPhyTimers) );

    /* Schedule Overflow Calback */
    pNextEvent = &mPhyTimers[0];
    pNextEvent->callback = PhyTime_OverflowCB;
    pNextEvent->timestamp = (gPhyTimerOverflow+1) << gPhyTimeShift_c;
    PhyTimeSetWaitTimeout( (uint32_t*)&pNextEvent->timestamp );

    return gPhyTimeOk_c;
}

/*! *********************************************************************************
* \brief  Returns a 64bit timestamp value to be used by the MAC Layer
*
* \return  phyTimeTimestamp_t PHY timestamp
*
********************************************************************************** */
phyTimeTimestamp_t PhyTime_GetTimestamp(void)
{
    phyTimeTimestamp_t time = 0;

    OSA_EnterCritical(kCriticalDisableInt);
    PhyTimeReadClock( (uint32_t*)&time );
    time |= (gPhyTimerOverflow << gPhyTimeShift_c);
    OSA_ExitCritical(kCriticalDisableInt);

    return time;
}

/*! *********************************************************************************
* \brief  Schedules an event
*
* \param[in]  pEvent  event to be scheduled
*
* \return  phyTimeTimerId_t  the id of the alocated timer
*
********************************************************************************** */
phyTimeTimerId_t PhyTime_ScheduleEvent( phyTimeEvent_t *pEvent )
{
    phyTimeTimerId_t tmr;

    /* Parameter validation */
    if( NULL == pEvent->callback )
    {
        return gInvalidTimerId_c;
    }

    /* Search for a free slot (slot 0 is reserved for the Overflow calback) */
    OSA_EnterCritical(kCriticalDisableInt);
    for( tmr=1; tmr<gMaxPhyTimers_c; tmr++ )
    {
        if( mPhyTimers[tmr].callback == NULL )
        {
            mPhyTimers[tmr] = *pEvent;
            break;
        }
    }
    OSA_ExitCritical(kCriticalDisableInt);

    if( tmr >= gMaxPhyTimers_c )
        return gInvalidTimerId_c;

    /* Program the next event */
    if((NULL == pNextEvent) ||
       (NULL != pNextEvent  && mPhyTimers[tmr].timestamp < pNextEvent->timestamp))
    {
        PhyTime_Maintenance();
    }

    return tmr;
}

/*! *********************************************************************************
* \brief  Cancel an event
*
* \param[in]  timerId  the Id of the timer
*
* \return  phyTimeStatus_t
*
********************************************************************************** */
phyTimeStatus_t PhyTime_CancelEvent( phyTimeTimerId_t timerId )
{
    if( (timerId == 0) || (timerId >= gMaxPhyTimers_c) || (NULL == mPhyTimers[timerId].callback) )
    {
        return gPhyTimeNotFound_c;
    }

    OSA_EnterCritical(kCriticalDisableInt);
    if( pNextEvent == &mPhyTimers[timerId] )
        pNextEvent = NULL;

    mPhyTimers[timerId].callback = NULL;
    OSA_ExitCritical(kCriticalDisableInt);

    return gPhyTimeOk_c;
}

/*! *********************************************************************************
* \brief  Cancel all event with the specified paameter
*
* \param[in]  param  event parameter
*
* \return  phyTimeStatus_t
*
********************************************************************************** */
phyTimeStatus_t PhyTime_CancelEventsWithParam ( uint32_t param )
{
    uint32_t i;
    phyTimeStatus_t status = gPhyTimeNotFound_c;

    OSA_EnterCritical(kCriticalDisableInt);
    for( i=1; i<gMaxPhyTimers_c; i++ )
    {
        if( mPhyTimers[i].callback && (param == mPhyTimers[i].parameter) )
        {
            status = gPhyTimeOk_c;
            mPhyTimers[i].callback = NULL;
            if( pNextEvent == &mPhyTimers[i] )
                pNextEvent = NULL;
        }
    }
    OSA_ExitCritical(kCriticalDisableInt);

    return status;
}

/*! *********************************************************************************
* \brief  Run the callback for the recently expired event
*
********************************************************************************** */
void PhyTime_RunCallback( void )
{
    uint32_t param;
    phyTimeCallback_t cb;

    if( pNextEvent )
    {
        OSA_EnterCritical(kCriticalDisableInt);

        param = pNextEvent->parameter;
        cb = pNextEvent->callback;
        pNextEvent->callback = NULL;
        pNextEvent = NULL;

        OSA_ExitCritical(kCriticalDisableInt);

        cb(param);
    }
}

/*! *********************************************************************************
* \brief  Expire events too close to be scheduled.
*         Program the next event
*
********************************************************************************** */
void PhyTime_Maintenance( void )
{
    phyTimeTimestamp_t currentTime;
    phyTimeEvent_t *pEv;

    PhyTimeDisableWaitTimeout();

    while(1)
    {
        OSA_EnterCritical(kCriticalDisableInt);
        
        pEv = PhyTime_GetNextEvent();
        currentTime = PhyTime_GetTimestamp();
        
        /* Program next event if exists */
        if( pEv )
        {
            pNextEvent = pEv;
            
            if( pEv->timestamp > (currentTime + gPhyTimeMinSetupTime_c) )
            {
                PhyTimeSetWaitTimeout( (uint32_t*)&pEv->timestamp );
                pEv = NULL;
            }
        }

        OSA_ExitCritical(kCriticalDisableInt);

        if( !pEv )
            break;

        PhyTime_RunCallback();
    }
    
}


/*! *********************************************************************************
* \brief  Timer Overflow callback
*
* \param[in]  param
*
********************************************************************************** */
static void PhyTime_OverflowCB( uint32_t param )
{
    (void)param;

    gPhyTimerOverflow++;

    mPhyTimers[0].callback = PhyTime_OverflowCB;
    mPhyTimers[0].timestamp = (gPhyTimerOverflow+1) << 24;
}

/*! *********************************************************************************
* \brief  Search for the next event to be scheduled
*
* \return phyTimeEvent_t pointer to the next event to be scheduled
*
********************************************************************************** */
static phyTimeEvent_t* PhyTime_GetNextEvent( void )
{
    phyTimeEvent_t *pEv = NULL;
    uint32_t i;

    /* Search for the next event to be serviced */
    for( i=0; i<gMaxPhyTimers_c; i++ )
    {
        if( NULL != mPhyTimers[i].callback )
        {
            if( NULL == pEv )
            {
                pEv = &mPhyTimers[i];
            }
            /* Check which event expires first */
            else if( mPhyTimers[i].timestamp < pEv->timestamp )
            {
                pEv = &mPhyTimers[i];
            }
        }
    }

    return pEv;
}
