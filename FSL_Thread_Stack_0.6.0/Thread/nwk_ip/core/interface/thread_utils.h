
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

#ifndef _THREAD_UTILS_H
#define _THREAD_UTILS_H
/*!=================================================================================================
\file       thread_utils.h
\brief      This is a header file for the Thread module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "thread_manager.h"

/*==================================================================================================
Public macros
==================================================================================================*/



/*==================================================================================================
Public type definitions
==================================================================================================*/



/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

void Thread_GetOwnIpAddress(ipAddr_t *pIpAddr, llAddrSize_t llAddrType,
                                           threadInstance_t *pThreadInstance);

void Thread_UnbindShortAddrIp(ifHandle_t* pIfHandle);

void Thread_UnbindGlobalAddrIp(ifHandle_t* pIfHandle, ipAddr_t *pPrefix, uint32_t prefixLen);

void Thread_GenerateChallenge(threadInstance_t *pThreadInstance);

/*!*************************************************************************************************
\fn void Thread_ResetAdvTrickleTmr
\brief Resets the tricke timer used for network advertisements

\retval      none
***************************************************************************************************/
void Thread_ResetAdvTrickleTmr(threadInstance_t *pThreadInstance);

ifHandle_t * Thread_GetIpIfByMacInstance(    instanceId_t macInstanceId);

void Thread_GetNodeUla(threadInstance_t *pThreadInstance,ipAddr_t *pIpAddr, uint64_t *pMacAddr,
                                   macAbsAddrModeType_t macAddrType);

void Thread_ProcessSourceAddress(uint16_t previousAddress, uint16_t newAddress, threadInstance_t *pThreadInstance);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _THREAD_UTILS_H */
