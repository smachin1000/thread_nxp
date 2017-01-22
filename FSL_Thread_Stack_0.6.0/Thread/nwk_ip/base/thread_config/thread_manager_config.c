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

/*!=================================================================================================
\file       thread_manager_config.c
\brief      This is a public source file for the Thread Manager module. It contains
            Thread configurations.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* FSL Framework */
/* 802.15.4 PHY-MAC */
/* IP Stack Lite */
/* Communication Interfaces */
/* Application */
#include "thread_manager_config.h"

#if STACK_THREAD

#include "sixlowpan_fwd_thread.h"
#include "mac_filtering.h"
#include "thread_manager.h"
/*==================================================================================================
Private macros
==================================================================================================*/

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/* MPL structures */
static CONST mplInstanceCfg_t mThreadMplCfg                  = {THREAD_STACK_MPL_CFG};
static CONST ipAddr_t mThreadMcastAddreses[]                 = {THREAD_STACK_MPL_ADDR};


static CONST securityMaterial_t mThreadSecMaterial                  = {THREAD_SECURITY_MATERIAL};


/* Thread Router */
#if THREAD_ROUTER
static CONST macFilteringConfig_t mThreadRouterICanHearYouTbl[]     = {THREAD_ROUTER_NEIGHBORS};
static CONST macCfg_t mThreadRouterMacCfg                           = {THREAD_ROUTER_MAC_CFG};
static CONST adpIb_t mThreadRouterAdpIb                             = {THREAD_ROUTER_ADP_IB};
static CONST stackConfig_t mThreadRouterCfg                         = {THREAD_ROUTER_CONFIGURATION};
#endif


static CONST macFilteringConfig_t mThreadICanHearYouTbl[]      = {THREAD_NODE_NEIGHBORS};
static CONST macCfg_t mThreadMacCfg                            = {THREAD_MAC_CFG};
static CONST adpIb_t mThreadAdpIb                              = {THREAD_ADP_IB};
static CONST stackConfig_t mThreadCfg                          = {THREAD_CONFIGURATION};

/* Active Stack Configuration */
CONST stackConfig_t* pStackCfg[] = {
                                            &mThreadCfg,
                                            NULL
                                          };

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
StackStartCfg_RegisterStatic(stackStartCfg, StartThread, (stackConfig_t**)pStackCfg);

/*==================================================================================================
Public functions
==================================================================================================*/

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Private debug functions
==================================================================================================*/

#endif /* STACK_THREAD */
