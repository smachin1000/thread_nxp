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

#ifndef _SIXLOWPAN_FWD_THREAD_H
#define _SIXLOWPAN_FWD_THREAD_H

/*!=================================================================================================
\file       sixlowpan_fwd_thread.h
\brief      This is a header file for the sixlowpan forwarding functionality for THREAD.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "network_utils.h"
#include "mac_abs_types.h"
#include "thread_cfg.h"

#if THR_ROUTING_ENABLE
/*==================================================================================================
Public macros
==================================================================================================*/
#define ROUTER_ID_MASK_BYTE         (0xFF <<(8 - THR_ROUTER_BITS_SIZE))
#define ROUTER_ID_MASK              (0xFF00 << (8 - THR_ROUTER_BITS_SIZE))
#define CHILD_ID_MASK               (0xFFFF >> THR_ROUTER_BITS_SIZE)

#define ROUTE_ELEM_UNASIGNED        0xF1
#define SLWPCFG_MAX_THREAD_ROUTERS  32

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*!*************************************************************************************************
\fn     bool_t adpThreadIsAddrOnLink(ipAddr_t *pIpAddr, uint32_t interfaceId)
\brief  Function used to determine the on link status of a destination address.

\param  [in]    pIpAddr         Pointer to the IPv6 address.
\param  [in]    interfaceId     Interface ID.

\retval         TRUE            Address is ON LINK.
\retval         FALSE           Address is OFF LINK.
***************************************************************************************************/
bool_t adpThreadIsAddrOnLink(ipAddr_t *pIpAddr, uint32_t interfaceId);

/*!*************************************************************************************************
\fn     llAddr_t adpThreadGetNextHop(llAddr_t* pMyAddr, llAddr_t* pFinalAddr, instanceId_t instanceId)
\brief  Function used to get the next hop Link-Layer address.

\param  [in]    pMyAddr         Pointer to device Link-Layer address.
\param  [in]    pFinalAddr      Pointer to final destination Link-Layer address.
\param  [in]    instanceId      Instance ID.

\return         llAddr_t        Next hop Link-Layer address.
***************************************************************************************************/
llAddr_t adpThreadGetNextHop(llAddr_t* myAddr, llAddr_t* finalAddr, instanceId_t instanceId);


bool_t adpThreadIsMeshNeeded(void * param);

/*!*************************************************************************************************
\fn     llAddr_t adpThreadGetParent(llAddr_t* pMyAddr, llAddr_t* pFinalAddr, instanceId_t instanceId)
\brief  Function used to get the parent node Link-Layer address.

\param  [in]    pMyAddr         Pointer to device Link-Layer address.
\param  [in]    pFinalAddr      Pointer to final destination Link-Layer address.
\param  [in]    instanceId      Instance ID.

\return         llAddr_t        Parent node Link-Layer address.
***************************************************************************************************/
llAddr_t adpThreadGetParent(llAddr_t* myAddr, llAddr_t* finalAddr, instanceId_t instanceId);

/*!*************************************************************************************************
\fn     void adpThreadMCastForward(macAbsMcpsDataInd_t * pMcpsDataInd)
\brief  Function used to forward multicast packets.

\param  [in]    pMcpsDataInd    Pointer to MCPS Data Indication.
***************************************************************************************************/
void adpThreadMCastForward(macAbsMcpsDataInd_t * pMcpsDataInd);

/*!*************************************************************************************************
\fn     void adpThreadSetRoutingTable(void * pMeshRouteTbl, instanceId_t instanceId)
\brief  Function used to set the whole routing table.

\param  [in]    pMeshRouteTbl   Pointer to routing table.
\param  [in]    instanceId      Instance ID.
***************************************************************************************************/
void adpThreadSetRoutingTable(void * pMeshRouteTbl, instanceId_t instanceId);

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
/*================================================================================================*/

#endif
#endif  /* _SIXLOWPAN_FWD_THREAD_H */
