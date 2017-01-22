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

#ifndef _THREAD_ROUTER_H
#define _THREAD_ROUTER_H

/*!=================================================================================================
\file       thread_manager.h
\brief      This is a header file for the Thread module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "stack_manager_if.h"

/*==================================================================================================
Public macros
==================================================================================================*/

                                        

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*! Thread routing Link set */
typedef struct thrLqCacheEntry_tag
{
    //uint8_t shortAddr[gLlayerAddrEui16_c];    
    uint16_t shortAddr ;    
    uint8_t eui[gLlayerAddrEui64_c];
}thrLqCacheEntry_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/



/*==================================================================================================
Public function prototypes
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

bool_t Thread_ResolveAddr(ipPktInfo_t* pIpPktInfo);

void Thread_ERouterHandler(mleCallbackParams_t *pMleCallbackParams, mleNeighbor_t *pMleNeighbor);

void Thread_RouterHandler(mleCallbackParams_t *pMleCallbackParams, mleNeighbor_t *pMleNeighbor);

void Thread_RouterNeighborSyncHandler(mleCallbackParams_t *pMleCallbackParams);

void Thread_ERouterContinueAttach(threadInstance_t *pThreadInstance);

/*!*************************************************************************************************
\fn     uint16_t Thread_NodeAddressGenerate(uint16_t routerAddress)
\brief  Private function for Thread Manager module. This function generates a Node Short Address
        based on Router Address.

         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
        | router ID |R| child index     |
        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

\param  [in]    routerAddress   router address

\return         uint16_t        node address
***************************************************************************************************/
uint16_t Thread_NodeAddressGenerate(threadInstance_t *pThreadInstance);

void Thread_RouterPromoteRoutine(void *pParam);

/*!*************************************************************************************************
\fn void Thread_RouterAssignedNotify(uint64_t extendedAddress, uint16_t routerId)
\brief Function called by the Thread Routing module when a new Router address has been assigned.

\param [in] extendedAddress         The extended address of the device.
\param [in] routerId                The new router Id that has been assigned to the device.

\retval      none
***************************************************************************************************/
void Thread_RouterAssignedNotify(uint64_t extendedAddress, uint16_t routerId);

void Thread_SetAddr(ifHandle_t* pIfHandle, ipAddr_t *ipAddr, void* pData);

#ifdef __cplusplus
}
#endif

/*================================================================================================*/

#endif  /* _THREAD_ROUTER_H */
