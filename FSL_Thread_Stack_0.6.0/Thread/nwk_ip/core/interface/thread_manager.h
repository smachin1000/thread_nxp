/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
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

#ifndef _THREAD_MANAGER_H
#define _THREAD_MANAGER_H
/*!=================================================================================================
\file       thread_manager.h
\brief      This is a header file for the stack_manager module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "stack_manager_if.h"

#include "thread_cfg.h"

#include "thread_state_machine.h"

#include "mle_basic.h"

#include "mle_extended.h"

#include "thread_manager.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/* Security */

#define THREAD_MAX_NB_OF_KEYS               2

#define THREAD_FIRST_KEY_IDX_IN_KEY_TABLE   0
                                            
#define THREAD_SECOND_KEY_IDX_IN_KEY_TABLE  1

#define THREAD_MLE_SECURITY_ENABLED         1

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*! Device types */
typedef enum threadDeviceType_tag
{
    gThreadDevTypeEndDevice_c = 0x00U,              /*!< Sleepy End Device, no routing capability */
    gThreadDevTypeEligibleRouter_c = 0x01U,         /*!< End Device which can become a Router */
    gThreadDevTypeRouter_c = 0x02U,                 /*!< Router Device capable of forwarding packets */  
} threadDevType_t;

/*! Device roles */
typedef enum threadDeviceRole_tag
{
    gThreadRoleNormalNode_c = 0x00U,
    gThreadRoleLeader_c = 0x01U
} threadDevRole_t;

typedef struct mleNeighbor_tag
{
    ///TODO: NVNG Notes: mleFrameCounter & timestamp.
    uint64_t              extendedAddress;          /*!< Extended Address */
    uint16_t              shortAddress;             /*!< Short Address */
    uint8_t               challenge[16];            /*!< Challenge sent by the parent selected */
    uint8_t               challengeLength;          /*!< Challenge length in bytes */
    uint32_t              linkLayerFrameCtr;        /*!< Link Layer Frame Counter */
    uint32_t              mleFrameCounter;          /*!< MLE Frame Counter */    
    uint32_t              timestamp;                /*!< Last Time of Communication */
    mleTlvConnectivity_t  connectivity;             /*!< Connectivity */
    uint8_t               RSSI;                     /*!< RSSI */
    uint8_t               mode;                     /*!< Device mode */
    threadSmStates_t      state;                    /*!< Neighbor State */
    bool_t                bIsParent;                /*!< Flag indicating if this is the parent */
} mleNeighbor_t;

typedef struct threadInstance_tag
{
    threadDevType_t       currentDevType;
    threadDevRole_t       currentDevRole;
    threadSmStates_t      threadState;
    stackConfig_t         *pStackCfg;
    mleTlvChallenge_t     *pMleLastTlvChallengeSent;
    uint32_t              aNodeAddrMask[(1<<THR_CHILD_BITS_SIZE) / 32];
    tmrTimerID_t          nodeTimerId;
    tmrTimerID_t          pollTmrId;
    void                  *trikleTmrId;
    ifHandle_t            *pIfHandle;
} threadInstance_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn void void StartThread(stackConfig_t *pStackCfg, taskMsgQueue_t *pTaskMsgQueue)
\brief

\param [in] pStackCfg       pointer to the stack configuration structure
\param [in] pTaskMsgQueue   pointer to the task message queue

\retval      none
***************************************************************************************************/
void StartThread(stackConfig_t *pStackCfg, taskMsgQueue_t *pTaskMsgQueue);

/*!*************************************************************************************************
\fn void Thread_MleNeighborGetUla16(uint64_t eui, ipAddr_t *pIpAddr, uint32_t *pTimestamp)
\brief Function called by the Thread Routing module when a new Router address has been assigned.

\param [in]  eui            The extended address of the device.
\param [out] pIpAddr        The new router Id that has been assigned to the device.
\param [out] pTimestamp     The new router Id that has been assigned to the device.

\retval     TRUE            ULA16 has been generated.
\retval     FALSE           ULA16 has not been generated.
***************************************************************************************************/
bool_t Thread_MleNeighborGetChildUla16(uint64_t eui, ipAddr_t *pIpAddr, uint32_t *pTimestamp);

void Thread_StartLeader(threadInstance_t *pThreadInstance);
void Thread_StartNode(stackConfig_t *pStackCfg, uint8_t scanMask);
void Thread_StartRouterIdDhcp();
void Thread_DataRequestProcess(mleCallbackParams_t *pMleCallbackParams, mleNeighbor_t *pMleNeighbor);

void Thread_DataResponseProcess(mleCallbackParams_t *pMleCallbackParams);

void Thread_TlvRequestProcess(mleOtaTlvTlvRequest_t *pTlvRequest, list_t *pTlvList, 
                                             ifHandle_t *pIfHandle, mleNeighbor_t *pMleNeighbor);

void Thread_LinkSyncProcess(mleCallbackParams_t *pMleCallbackParams, mleNeighbor_t *pMleNeighbor);                                             

void Thread_NwkAdvProcess(mleCallbackParams_t *pMleCallbackParams);

void Thread_NwkAdvSend(void *pParam);

mleNeighbor_t * Thread_MleNeighborAdd(llAddr_t address, threadSmStates_t state);

mleNeighbor_t * Thread_MleNeighborGet(ipAddr_t *pNeighborIp);

uint32_t Thread_MleNeighborGetIdx(mleNeighbor_t *pMleNeighbor);

mleNeighbor_t * Thread_GetParent(threadInstance_t *pThreadInstance);

void Thread_MleResetSecurityCounters(void);

threadInstance_t * Thread_GetInstanceByIf(ifHandle_t *pIfHandle);
void Thread_LeaderIpConfig(threadInstance_t *pThreadInstance,stackConfig_t *pStackCfg);


#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _THREAD_MANAGER_H */
