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

#ifndef _PMTU_DISCOVERY_H
#define _PMTU_DISCOVERY_H
/*!=================================================================================================
\file       pmtu_discovery.h
\brief      This is a header file for the Neighbor Discovery for IP version 6 (IPv6) module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"

/* Network Includes */
#include "ip.h"
#include "network_utils.h" 

/*==================================================================================================
Public macros
==================================================================================================*/



/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef void (*pmtuNotifyDroppedPktCb_t)(ipAddr_t *pDestAddr);
typedef void (*pmtuNotifyIncreasePmtuCb_t)(ipAddr_t *pDestAddr, uint32_t newPmtu);

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
\fn     uint32_t PMTU_GetPmtu(ifHandle_t* ifHandle, ipAddr_t *pSrcIpAddr,ipAddr_t *pDestIpAddr)
\brief  Interface function for the Path MTU discovery module.

\param  [in]    ifHandle        Interface Handle. If the Sourcer Address is unspecified, the ifHandle MUST NOT be NULL.
\param  [in]    pSrcIpAddr      Pointer to the source IP address.
\param  [in]    pDestIpAddr     Pointer to the destination IP address.

\return         uint32_t        Value of the PMTU for the specified path.
***************************************************************************************************/
uint32_t PMTU_GetPmtu(ifHandle_t* ifHandle, ipAddr_t *pSrcIpAddr, ipAddr_t *pDestIpAddr);

#if ND_ENABLED

/*!*************************************************************************************************
\fn     void PMTU_Init(void)
\brief  Interface function for the Path MTU discovery module.
***************************************************************************************************/
void PMTU_Init(void);

#if PMTU_MANAGEMENT_FUNCTIONS_ENABLED

/*!*************************************************************************************************
\fn     void PMTU_SetPmtuDiscovery(void)
\brief  Interface function for the Path MTU discovery module.

\param  [in]    pDestIpAddr     Pointer to the destination IP address.
\param  [in]    enabled         TRUE/FALSE - Enables/Disables Path MTU discovery for this path.
***************************************************************************************************/
void PMTU_SetPmtuDiscovery(ipAddr_t *pDestIpAddr, bool_t enabled);

/*!*************************************************************************************************
\fn     void PMTU_SetPmtuIncreaseInterval(uint32_t increaseInterval)
\brief  Interface function for the Path MTU discovery module.

\param  [in]    increaseInterval    Pointer to the destination IP address.
***************************************************************************************************/
void PMTU_SetPmtuIncreaseInterval(uint32_t increaseInterval);

#endif /* PMTU_MANAGEMENT_FUNCTIONS_ENABLED */

#if PMTU_CALLBACKS_ENABLED

/*!*************************************************************************************************
\fn     void PMTU_GetPmtu(void)
\brief  Interface function for the Path MTU discovery module.

\return uint32_t    Value of the PMTU for the specified path.
***************************************************************************************************/
void PMTU_RegisterCallbacks(pmtuNotifyDroppedPktCb_t droppedPktCb, pmtuNotifyIncreasePmtuCb_t increasePmtuCb);

#endif /* PMTU_CALLBACKS_ENABLED */

/*!*************************************************************************************************
\fn     void PMTU_TimerHandler(uint32_t timerPeriodInSec)

\brief  Path MTU Discovery periodical timer handler.

\param  [in]    timerPeriodInSec      The interval the timer is triggered.
***************************************************************************************************/
void PMTU_TimerHandler(uint32_t timerPeriodInSec);

#endif /* ND_ENABLED */

#ifdef __cplusplus
}
#endif

/*================================================================================================*/
#endif  /* _PMTU_DISCOVERY_H */

