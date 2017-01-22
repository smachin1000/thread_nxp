/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file PhyStateMachine.c
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


#ifdef gSrcTask_d
#undef gSrcTask_d
#endif

#define gSrcTask_d PHY


/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"

#include "PhyInterface.h"
#include "Phy.h"
#include "MemManager.h"
#include "Messaging.h"
#include "Panic.h"
#include "FunctionLib.h"

#include "MCR20Drv.h"
#include "MCR20Reg.h"

#include "AspInterface.h"
#include "MpmInterface.h"


/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/
#define mPhyMaxIdleRxDuration_c      (0xF00000) /* [sym] */

#define ProtectFromXcvrInterrupt()   ProtectFromMCR20Interrupt()
#define UnprotectFromXcvrInterrupt() UnprotectFromMCR20Interrupt()

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void Phy24Task(Phy_PhyLocalStruct_t *pPhyData);

static phyStatus_t Phy_HandlePdDataReq( Phy_PhyLocalStruct_t *pPhyData, macToPdDataMessage_t * pMsg );

static void Phy_EnterIdle( Phy_PhyLocalStruct_t *pPhyData );

static void PLME_SendMessage(Phy_PhyLocalStruct_t *pPhyData, phyMessageId_t msgType);

static void PD_SendMessage(Phy_PhyLocalStruct_t *pPhyData, phyMessageId_t msgType);

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
Phy_PhyLocalStruct_t phyLocal[gPhyInstancesCnt_c];
extern volatile uint32_t mPhySeqTimeout;

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  This function creates the PHY task
*
********************************************************************************** */
void Phy_Init(void)
{
    uint32_t i;

    PhyHwInit();
    ASP_Init( 0, gAspInterfaceId );
    MPM_Init();

    for( i=0; i<gPhyInstancesCnt_c; i++ )
    {
        phyLocal[i].flags = 0;
        phyLocal[i].rxParams.pRxData = NULL;

        /* Prepare input queues.*/
        MSG_InitQueue( &phyLocal[i].macPhyInputQueue );
    }

    PhyIsrPassRxParams( NULL );
    PhyPlmeSetPwrState( gPhyDefaultIdlePwrMode_c );
}

/*! *********************************************************************************
* \brief  This function binds a MAC instance to a PHY instance
*
* \param[in]  instanceId The instance of the MAC
*
* \return  The instance of the PHY.
*
********************************************************************************** */
instanceId_t BindToPHY( instanceId_t macInstance )
{
    return 0;
}

/*! *********************************************************************************
* \brief  This function registers the MAC PD and PLME SAP handlers
*
* \param[in]  pPD_MAC_SapHandler   Pointer to the MAC PD handler function
* \param[in]  pPLME_MAC_SapHandler Pointer to the MAC PLME handler function
* \param[in]  instanceId           The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
void Phy_RegisterSapHandlers( PD_MAC_SapHandler_t pPD_MAC_SapHandler,
                              PLME_MAC_SapHandler_t pPLME_MAC_SapHandler,
                              instanceId_t instanceId )
{
    phyLocal[instanceId].PD_MAC_SapHandler = pPD_MAC_SapHandler;
    phyLocal[instanceId].PLME_MAC_SapHandler = pPLME_MAC_SapHandler;
}

/*! *********************************************************************************
* \brief  This function represents the PHY's task
*
* \param[in]  taskParam The instance of the PHY
*
********************************************************************************** */
static void Phy24Task(Phy_PhyLocalStruct_t *pPhyStruct)
{
    uint8_t state;
    phyMessageHeader_t * pMsgIn;
    phyStatus_t status = gPhySuccess_c;

    ProtectFromXcvrInterrupt();
    state = PhyGetSeqState();

    /* Handling messages from upper layer */
    while( MSG_Pending(&pPhyStruct->macPhyInputQueue) )
    {
        /* PHY doesn't free dynamic alocated messages! */
        pMsgIn = MSG_DeQueue( &pPhyStruct->macPhyInputQueue );
        pPhyStruct->currentMacInstance = pMsgIn->macInstance;

        if( gRX_c == state )
        {
            uint8_t phyReg = MCR20Drv_DirectAccessSPIRead(SEQ_STATE) & 0x1F;

            if( phyReg <= 0x06 || phyReg == 0x15 || phyReg == 0x16 )
            {
                MSG_QueueHead( &pPhyStruct->macPhyInputQueue, pMsgIn );
                UnprotectFromXcvrInterrupt();
                return;
            }
            else if( pPhyStruct->flags & gPhyFlagIdleRx_c )
            {
                PhyPlmeForceTrxOffRequest();
                state = gIdle_c;
            }
        }
        else if( gIdle_c != state )
        {
            /* try again later */
            MSG_QueueHead( &pPhyStruct->macPhyInputQueue, pMsgIn );
            UnprotectFromXcvrInterrupt();
            return;
        }

#if gMpmIncluded_d
        if( status == gPhySuccess_c )
        {
            status = MPM_PrepareForTx( pMsgIn->macInstance );
        }
#endif

        if( status == gPhySuccess_c )
        {
            pPhyStruct->flags &= ~(gPhyFlagIdleRx_c);

            switch( pMsgIn->msgType )
            {
            case gPdDataReq_c:
                status = Phy_HandlePdDataReq( pPhyStruct, (macToPdDataMessage_t *)pMsgIn );
                break;
            case gPlmeCcaReq_c:
                status = PhyPlmeCcaEdRequest(gPhyCCAMode1_c, gPhyContCcaDisabled);
                break;
            case gPlmeEdReq_c:
                status = PhyPlmeCcaEdRequest(gPhyEnergyDetectMode_c, gPhyContCcaDisabled);
                break;
            default:
                status = gPhyInvalidPrimitive_c;
            }
        }

        /* Check status */
        if( gPhySuccess_c == status )
        {
            UnprotectFromXcvrInterrupt();
            return;
        }
        else
        {
            switch( pMsgIn->msgType )
            {
            case gPdDataReq_c:
                if( ((macToPdDataMessage_t*)pMsgIn)->msgData.dataReq.CCABeforeTx == gPhyNoCCABeforeTx_c )
                {
                    PD_SendMessage(pPhyStruct, gPdDataCnf_c);
                    break;
                }
                /* Fallthorough */
            case gPlmeCcaReq_c:
                pPhyStruct->channelParams.channelStatus = gPhyChannelBusy_c;
                PLME_SendMessage(pPhyStruct, gPlmeCcaCnf_c);
                break;
            case gPlmeEdReq_c:
                pPhyStruct->channelParams.energyLeveldB = 0;
                PLME_SendMessage(pPhyStruct, gPlmeEdCnf_c);
                break;
            default:
                PLME_SendMessage(pPhyStruct, gPlmeTimeoutInd_c);
            }
        }
    }/* while( MSG_Pending(&pPhyStruct->macPhyInputQueue) ) */

    UnprotectFromXcvrInterrupt();

    /* Check if PHY can enter Idle state */
    if( gIdle_c == state )
    {
        Phy_EnterIdle( pPhyStruct );
    }
}

/*! *********************************************************************************
* \brief  This is the PD SAP message handler
*
* \param[in]  pMsg Pointer to the PD request message
* \param[in]  instanceId The instance of the PHY
*
* \return  The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PD_SapHandler(macToPdDataMessage_t *pMsg, instanceId_t phyInstance)
{
    phyStatus_t result = gPhySuccess_c;
    uint8_t baseIndex = 0;

    if( NULL == pMsg )
    {
        return gPhyInvalidParameter_c;
    }

#if gMpmIncluded_d
    if( pMsg->msgType == gPdIndQueueInsertReq_c || pMsg->msgType == gPdIndQueueRemoveReq_c )
    {
        baseIndex = MPM_GetRegSet( MPM_GetPanIndex( pMsg->macInstance ) ) *
                   (gPhyIndirectQueueSize_c/gMpmPhyPanRegSets_c);
    }
#endif

    switch( pMsg->msgType )
    {
    case gPdIndQueueInsertReq_c:
        result = PhyPp_IndirectQueueInsert(baseIndex + pMsg->msgData.indQueueInsertReq.index,
                                           pMsg->msgData.indQueueInsertReq.checksum,
                                           phyInstance);
        break;

    case gPdIndQueueRemoveReq_c:
        result = PhyPp_RemoveFromIndirect(baseIndex + pMsg->msgData.indQueueRemoveReq.index,
                                          phyInstance);
        break;

    case gPdDataReq_c:
        MSG_Queue(&phyLocal[phyInstance].macPhyInputQueue, pMsg);
        Phy24Task( &phyLocal[phyInstance] );
        break;

    default:
        result = gPhyInvalidPrimitive_c;
    }

    return result;
}

/*! *********************************************************************************
* \brief  This is the PLME SAP message handler
*
* \param[in]  pMsg Pointer to the PLME request message
* \param[in]  instanceId The instance of the PHY
*
* \return  phyStatus_t The status of the operation.
*
********************************************************************************** */
phyStatus_t MAC_PLME_SapHandler(macToPlmeMessage_t * pMsg, instanceId_t phyInstance)
{
    Phy_PhyLocalStruct_t *pPhyStruct = &phyLocal[phyInstance];
    uint8_t phyRegSet = 0;
#if gMpmIncluded_d
    phyStatus_t result;
    int32_t panIdx = MPM_GetPanIndex( pMsg->macInstance );

    phyRegSet = MPM_GetRegSet( panIdx );
#endif

    if( NULL == pMsg )
    {
        return gPhyInvalidParameter_c;
    }

    switch( pMsg->msgType )
    {
    case gPlmeEdReq_c:
    case gPlmeCcaReq_c:
        MSG_Queue(&phyLocal[phyInstance].macPhyInputQueue, pMsg);
        Phy24Task( &phyLocal[phyInstance] );
        break;

    case gPlmeSetReq_c:
#if gMpmIncluded_d
        result = MPM_SetPIB(pMsg->msgData.setReq.PibAttribute,
                            &pMsg->msgData.setReq.PibAttributeValue,
                            panIdx );
        if( !MPM_isPanActive(panIdx) )
        {
            return result;
        }
#endif
        return PhyPlmeSetPIBRequest(pMsg->msgData.setReq.PibAttribute, pMsg->msgData.setReq.PibAttributeValue, phyRegSet, phyInstance);

    case gPlmeGetReq_c:
#if gMpmIncluded_d
        if( gPhySuccess_c == MPM_GetPIB(pMsg->msgData.getReq.PibAttribute, pMsg->msgData.getReq.pPibAttributeValue, panIdx) )
        {
            break;
        }
#endif
        return PhyPlmeGetPIBRequest( pMsg->msgData.getReq.PibAttribute, pMsg->msgData.getReq.pPibAttributeValue, phyRegSet, phyInstance);

    case gPlmeSetTRxStateReq_c:
        if(gPhySetRxOn_c == pMsg->msgData.setTRxStateReq.state)
        {
            if( PhyIsIdleRx(phyInstance) )
            {
                PhyPlmeForceTrxOffRequest();
            }
            else if( gIdle_c != PhyGetSeqState() )
            {
                return gPhyBusy_c;
            }
#if gMpmIncluded_d
            /* If another PAN has the RxOnWhenIdle PIB set, enable the DualPan Auto mode */
            if( gPhySuccess_c != MPM_PrepareForRx( pMsg->macInstance ) )
                return gPhyBusy_c;
#endif
            pPhyStruct->flags &= ~(gPhyFlagIdleRx_c);
            Phy_SetSequenceTiming(pMsg->msgData.setTRxStateReq.startTime,
                                  pMsg->msgData.setTRxStateReq.rxDuration);

            return PhyPlmeRxRequest(pMsg->msgData.setTRxStateReq.slottedMode, (phyRxParams_t *) &pPhyStruct->rxParams);
        }
        else if (gPhyForceTRxOff_c == pMsg->msgData.setTRxStateReq.state)
        {
#if gMpmIncluded_d
            if( !MPM_isPanActive(panIdx) )
                return gPhySuccess_c;
#endif
            pPhyStruct->flags &= ~(gPhyFlagIdleRx_c);
            PhyPlmeForceTrxOffRequest();
        }
        break;

    default:
        return gPhyInvalidPrimitive_c;
    }

    return gPhySuccess_c;
}

/*! *********************************************************************************
* \brief  This function programs a new TX sequence
*
* \param[in]  pMsg Pointer to the PD request message
* \param[in]  pPhyData pointer to PHY data
*
* \return  The status of the operation.
*
********************************************************************************** */
static phyStatus_t Phy_HandlePdDataReq( Phy_PhyLocalStruct_t *pPhyData, macToPdDataMessage_t * pMsg )
{
  phyStatus_t status = gPhySuccess_c;
  uint32_t time;

  if( NULL == pMsg->msgData.dataReq.pPsdu )
  {
    return gPhyInvalidParameter_c;
  }

  ProtectFromXcvrInterrupt();

  if( pMsg->msgData.dataReq.startTime != gPhySeqStartAsap_c )
  {
      PhyTimeSetEventTrigger( (uint16_t) pMsg->msgData.dataReq.startTime );
  }

  status = PhyPdDataRequest(&pMsg->msgData.dataReq , &pPhyData->rxParams, &pPhyData->txParams);

  if( pMsg->msgData.dataReq.txDuration != gPhySeqStartAsap_c )
  {
      OSA_EnterCritical(kCriticalDisableInt);
      PhyTimeReadClock( &time );
      time += pMsg->msgData.dataReq.txDuration;
      /* Compensate PHY overhead, including WU time */
      time += 54;
      PhyTimeSetEventTimeout( &time );
      OSA_ExitCritical(kCriticalDisableInt);
  }

  UnprotectFromXcvrInterrupt();

  if( gPhySuccess_c != status )
  {
    PhyTimeDisableEventTrigger();
    PhyTimeDisableEventTimeout();
  }

  return status;
}

/*! *********************************************************************************
* \brief  This function sets the start time and the timeout value for a sequence.
*
* \param[in]  startTime The absolute start time for the sequence.
*             If startTime is gPhySeqStartAsap_c, the start timer is disabled.
* \param[in]  seqDuration The duration of the sequence.
*             If seqDuration is 0xFFFFFFFF, the timeout is disabled.
*
********************************************************************************** */
void Phy_SetSequenceTiming(uint32_t startTime, uint32_t seqDuration)
{
    uint32_t endTime;

    OSA_EnterCritical(kCriticalDisableInt);

    if( gPhySeqStartAsap_c == startTime )
    {
        PhyTimeReadClock( &endTime );
    }
    else
    {
        PhyTimeSetEventTrigger( (uint16_t) startTime );
        endTime = startTime & gPhyTimeMask_c;
    }

    if( 0xFFFFFFFF != seqDuration )
    {
        endTime += seqDuration;
        endTime = endTime & gPhyTimeMask_c;

        PhyTimeSetEventTimeout( &(endTime) );
    }

    OSA_ExitCritical(kCriticalDisableInt);
}

/*! *********************************************************************************
* \brief  This function starts the IdleRX if the PhyRxOnWhenIdle PIB is set
*
* \param[in]  pPhyData pointer to PHY data
*
********************************************************************************** */
void Phy_EnterIdle( Phy_PhyLocalStruct_t *pPhyData )
{
    if( (pPhyData->flags & gPhyFlagRxOnWhenIdle_c)
#if gMpmIncluded_d
       /* Prepare the Active PAN/PANs */
       && (gPhySuccess_c == MPM_PrepareForRx(gInvalidInstanceId_c))
#endif
      )
    {
        pPhyData->flags |= gPhyFlagIdleRx_c;
        Phy_SetSequenceTiming( gPhySeqStartAsap_c, mPhyMaxIdleRxDuration_c );
        (void)PhyPlmeRxRequest( gPhyUnslottedMode_c, (phyRxParams_t*)&pPhyData->rxParams );
    }
    else
    {
        pPhyData->flags &= ~(gPhyFlagIdleRx_c);
    }
}

/*! *********************************************************************************
* \brief  This function sets the value of the maxFrameWaitTime PIB
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  time The maxFrameWaitTime value
*
********************************************************************************** */
void PhyPlmeSetFrameWaitTime( uint32_t time, instanceId_t instanceId )
{
    phyLocal[instanceId].maxFrameWaitTime = time;
}

/*! *********************************************************************************
* \brief  This function sets the state of the PhyRxOnWhenIdle PIB
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  state The PhyRxOnWhenIdle value
*
********************************************************************************** */
void PhyPlmeSetRxOnWhenIdle( bool_t state, instanceId_t instanceId )
{
    uint8_t radioState = PhyGetSeqState();
#if gMpmIncluded_d
    /* Check if at least one PAN has RxOnWhenIdle set */
    if( FALSE == state )
    {
        uint32_t i;

        for( i=0; i<gMpmMaxPANs_c; i++ )
        {
            MPM_GetPIB( gPhyPibRxOnWhenIdle, &state, i );
            if( state )
                break;
        }
    }
#endif
    if( state )
    {
        phyLocal[instanceId].flags |= gPhyFlagRxOnWhenIdle_c;
        if( radioState == gIdle_c)
        {
            Phy_EnterIdle( &phyLocal[instanceId] );
        }
#if gMpmIncluded_d
        else if( (radioState == gRX_c) && (phyLocal[instanceId].flags & gPhyFlagIdleRx_c) )
        {
            PhyPlmeForceTrxOffRequest();
            Phy_EnterIdle( &phyLocal[instanceId] );
        }
#endif
    }
    else
    {
        phyLocal[instanceId].flags &= ~gPhyFlagRxOnWhenIdle_c;
        if( (radioState == gRX_c) && (phyLocal[instanceId].flags & gPhyFlagIdleRx_c) )
        {
            PhyPlmeForceTrxOffRequest();
            phyLocal[instanceId].flags &= ~gPhyFlagIdleRx_c;
        }
    }
}

/*! *********************************************************************************
* \brief  This function starts the IdleRX if the PhyRxOnWhenIdle PIB is set
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
bool_t PhyIsIdleRx( instanceId_t instanceId )
{
    if( (phyLocal[instanceId].flags & gPhyFlagIdleRx_c) && (gRX_c == PhyGetSeqState()))
        return TRUE;

    return FALSE;
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a TX operation completed successfully.
*         If the received ACK has FP=1, then the radio will enter RX state for
*         maxFrameWaitTime duration.
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  framePending The value of the framePending bit for the received ACK
*
********************************************************************************** */
void Radio_Phy_PdDataConfirm(instanceId_t instanceId, bool_t framePending)
{
    PhyTimeDisableEventTimeout();

    if( framePending )
    {
        phyLocal[instanceId].flags |= gPhyFlagFramePending_c;
        if( phyLocal[instanceId].maxFrameWaitTime > 0 )
        {
            /* Restart Rx asap if an ACK with FP=1 is received */
            phyLocal[instanceId].flags &= ~(gPhyFlagIdleRx_c);
            PhyPlmeRxRequest( gPhyUnslottedMode_c, (phyRxParams_t *) &phyLocal[instanceId].rxParams );
            Phy_SetSequenceTiming( gPhySeqStartAsap_c, phyLocal[instanceId].maxFrameWaitTime );
        }
    }
    else
    {
        phyLocal[instanceId].flags &= ~gPhyFlagFramePending_c;
    }

    PD_SendMessage(&phyLocal[instanceId], gPdDataCnf_c);
    Phy24Task(&phyLocal[instanceId]);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that new data has been received
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PdDataIndication(instanceId_t instanceId)
{
    PhyTimeDisableEventTimeout();

    PD_SendMessage(&phyLocal[instanceId], gPdDataInd_c);
    Phy24Task(&phyLocal[instanceId]);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that timer1 compare match occured
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_TimeWaitTimeoutIndication(instanceId_t instanceId)
{
    extern void (*gpfPhyTimeNotify)(void);

    if( gpfPhyTimeNotify )
    {
        gpfPhyTimeNotify();
    }
    else
    {
        PhyTime_RunCallback();
        PhyTime_Maintenance();
    }
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a CCA sequence has finished
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  phyChannelStatus The status of the channel: Idle/Busy
*
* \return  None.
*
********************************************************************************** */
void Radio_Phy_PlmeCcaConfirm(phyStatus_t phyChannelStatus, instanceId_t instanceId)
{
    PhyTimeDisableEventTimeout();

    phyLocal[instanceId].channelParams.channelStatus = phyChannelStatus;

    PLME_SendMessage(&phyLocal[instanceId], gPlmeCcaCnf_c);
    Phy24Task(&phyLocal[instanceId]);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a ED sequence has finished
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  energyLevel The enetgy level on the channel.
* \param[in]  energyLeveldB The energy level in DB
*
********************************************************************************** */
void Radio_Phy_PlmeEdConfirm(uint8_t energyLeveldB, instanceId_t instanceId)
{
    PhyTimeDisableEventTimeout();

    phyLocal[instanceId].channelParams.energyLeveldB = energyLeveldB;

    PLME_SendMessage(&phyLocal[instanceId], gPlmeEdCnf_c);
    Phy24Task(&phyLocal[instanceId]);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that the programmed sequence has timed out
*         The Radio is forced to Idle.
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_TimeRxTimeoutIndication(instanceId_t instanceId)
{
    if( !(phyLocal[instanceId].flags & gPhyFlagIdleRx_c) )
        PLME_SendMessage(&phyLocal[instanceId], gPlmeTimeoutInd_c);

    Phy24Task(&phyLocal[instanceId]);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that the programmed sequence has started
*
* \param[in]  instanceId The instance of the PHY
*
* \return  None.
*
********************************************************************************** */
void Radio_Phy_TimeStartEventIndication(instanceId_t instanceId)
{
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_StartEventInd_c);
    Phy24Task(&phyLocal[instanceId]);
#endif
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a SFD was detected.
*         Also, if there is not enough time to receive the entire packet, the
*         RX timeout will be extended.
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  frameLen the length of the PSDU
*
********************************************************************************** */
void Radio_Phy_PlmeRxSfdDetect(instanceId_t instanceId, uint32_t frameLen)
{
#if 1
    uint32_t currentTime;
    uint32_t time;

    OSA_EnterCritical(kCriticalDisableInt);

    //Read currentTime and Timeout values [sym]
    PhyTimeReadClock(&currentTime);

    frameLen = frameLen * 2 + 12 + 22 + 2; //Convert to symbols and add IFS and ACK duration

    if( mPhySeqTimeout > currentTime )
    {
        time = mPhySeqTimeout - currentTime;
    }
    else
    {
        time = (gPhyTimeMask_c - currentTime + mPhySeqTimeout) & gPhyTimeMask_c;
    }

    if( time > 4 )
    {
        mPhySeqTimeout = (currentTime + frameLen) & gPhyTimeMask_c;
        MCR20Drv_DirectAccessSPIMultiByteWrite( T3CMP_LSB, (uint8_t *)&mPhySeqTimeout, 3);
    }

    OSA_ExitCritical(kCriticalDisableInt);
#endif

#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_RxSfdDetectInd_c);
    Phy24Task(&phyLocal[instanceId]);
#endif
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a Sync Loss occured (PLL unlock)
*         The Radio is forced to Idle.
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PlmeSyncLossIndication(instanceId_t instanceId)
{
    PhyPlmeForceTrxOffRequest();
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_SyncLossInd_c);
#endif
    Radio_Phy_TimeRxTimeoutIndication(instanceId);
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that a Filter Fail occured
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_PlmeFilterFailRx(instanceId_t instanceId)
{
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_FilterFailInd_c);
    Phy24Task(&phyLocal[instanceId]);
#endif
}

/*! *********************************************************************************
* \brief  This function signals the PHY task that an unexpected Transceiver Reset
*          occured and force the TRX to Off
*
* \param[in]  instanceId The instance of the PHY
*
********************************************************************************** */
void Radio_Phy_UnexpectedTransceiverReset(instanceId_t instanceId)
{
    PhyPlmeForceTrxOffRequest();
#ifdef MAC_PHY_DEBUG
    PLME_SendMessage(&phyLocal[instanceId], gPlme_UnexpectedRadioResetInd_c);
#endif
    Radio_Phy_TimeRxTimeoutIndication(instanceId);
}

/*! *********************************************************************************
* \brief  Senf a PLME message to upper layer
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  msgType    The type of message to be sent
*
********************************************************************************** */
static void PLME_SendMessage(Phy_PhyLocalStruct_t *pPhyStruct, phyMessageId_t msgType)
{
    plmeToMacMessage_t * pMsg = MEM_BufferAlloc(sizeof(plmeToMacMessage_t));

    if(NULL == pMsg)
    {
        panic(0,(uint32_t)PLME_SendMessage,0,msgType);
        return;
    }

    pMsg->msgType = msgType;

    switch(msgType)
    {
    case gPlmeCcaCnf_c:
        pMsg->msgData.ccaCnf.status = pPhyStruct->channelParams.channelStatus;
        break;

    case gPlmeEdCnf_c:
        pMsg->msgData.edCnf.status        = gPhySuccess_c;
        pMsg->msgData.edCnf.energyLeveldB = pPhyStruct->channelParams.energyLeveldB;
        pMsg->msgData.edCnf.energyLevel   = Phy_GetEnergyLevel(pPhyStruct->channelParams.energyLeveldB);
        break;

    default:
        /* No aditional info needs to be filled */
        break;
    }

    pPhyStruct->PLME_MAC_SapHandler(pMsg, pPhyStruct->currentMacInstance);
}

/*! *********************************************************************************
* \brief  Senf a PD message to upper layer
*
* \param[in]  instanceId The instance of the PHY
* \param[in]  msgType    The type of message to be sent
*
********************************************************************************** */
static void PD_SendMessage(Phy_PhyLocalStruct_t *pPhyStruct, phyMessageId_t msgType)
{
    pdDataToMacMessage_t *pMsg;

    if( msgType == gPdDataInd_c )
    {
        uint32_t temp;
        uint16_t len = pPhyStruct->rxParams.psduLength - 2; //Excluding FCS (2 bytes);

        pMsg = pPhyStruct->rxParams.pRxData;
        pPhyStruct->rxParams.pRxData = NULL;

#if !gUsePBTransferThereshold_d
        MCR20Drv_PB_SPIBurstRead( (uint8_t *)(pMsg->msgData.dataInd.pPsdu), len );
#endif

        pMsg->msgType                         = gPdDataInd_c;
        pMsg->msgData.dataInd.ppduLinkQuality = pPhyStruct->rxParams.linkQuality;
        pMsg->msgData.dataInd.psduLength      = len;

        pMsg->msgData.dataInd.timeStamp       = PhyTime_GetTimestamp();      //current timestamp (64bit)
        temp = (uint32_t)(pMsg->msgData.dataInd.timeStamp & gPhyTimeMask_c); //convert to 24bit
        pMsg->msgData.dataInd.timeStamp -= (temp - pPhyStruct->rxParams.timeStamp) & gPhyTimeMask_c;
#if !(gMpmIncluded_d)
        pPhyStruct->PD_MAC_SapHandler(pMsg, pPhyStruct->currentMacInstance);
#else
        {
            uint32_t i, bitMask = PhyPpGetPanOfRxPacket();

            for( i=0; i<gMpmPhyPanRegSets_c; i++ )
            {
                if( bitMask & (1 << i) )
                {
                    bitMask &= ~(1 << i);
                    pPhyStruct->currentMacInstance = MPM_GetMacInstanceFromRegSet(i);

                    /* If the packet passed filtering on muliple PANs, send a copy to each one */
                    if( bitMask )
                    {
                        pdDataToMacMessage_t *pDataIndCopy;

                        pDataIndCopy = MEM_BufferAlloc(sizeof(pdDataToMacMessage_t) + len);
                        if( pDataIndCopy )
                        {
                            FLib_MemCpy(pDataIndCopy, pMsg, sizeof(pdDataToMacMessage_t) + len);
                            pPhyStruct->PD_MAC_SapHandler(pDataIndCopy, pPhyStruct->currentMacInstance);
                        }
                    }
                    else
                    {
                        pPhyStruct->PD_MAC_SapHandler(pMsg, pPhyStruct->currentMacInstance);
                        break;
                    }
                }
            }
        }
#endif
    }
    else
    {
        pMsg = MEM_BufferAlloc( sizeof(pdDataToMacMessage_t) );

        if(NULL == pMsg)
        {
            panic(0,(uint32_t)PD_SendMessage,0,gPdDataCnf_c);
            return;
        }

        pMsg->msgType = gPdDataCnf_c;

        if( pPhyStruct->flags & gPhyFlagFramePending_c )
        {
            pPhyStruct->flags &= ~(gPhyFlagFramePending_c);
            pMsg->msgData.dataCnf.status = gPhyFramePending_c;
        }
        else
        {
            pMsg->msgData.dataCnf.status = gPhySuccess_c;
        }

        pPhyStruct->PD_MAC_SapHandler(pMsg, pPhyStruct->currentMacInstance);
    }
}