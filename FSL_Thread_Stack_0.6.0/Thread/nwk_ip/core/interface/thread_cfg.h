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

#ifndef _THREAD_STACK_CFG_H
#define _THREAD_STACK_CFG_H
/*!=================================================================================================
\file       thread_stack_cfg.h
\brief      This configuration header file for the Thread stack parameters. It contains default values for
            module configuration defines.
\details    This file contains the folowing configuration options:

            THREAD_TASK_MSG_QUEUE_SIZE                          configurable value
            THREAD_TASK_STACK_SIZE                              configurable value
            THR_ROUTING_ENABLE                                         0|1
            THR_WEIGHTING_FACTOR                                      8 or 16
            THR_ID_REUSE_DELAY_SEC                              configurable value
            THR_ID_SEQUENCE_PERIOD_SEC                          configurable value
            THR_NETWORK_ID_TIMEOUT_SEC                          configurable value
            THR_TIMER_PERIOD_SEC                                configurable value
            THR_MAX_ROUTE_COST                                  configurable value
            THR_MAX_ROUTER_ID                                   configurable value
            THR_MAX_ALLOWED_ROUTERS                             configurable value
            THR_LEASE_QUERRY_CACHE_TBL_SIZE                     configurable value
            THR_MAX_NEIGHBOR_AGE                                configurable value
            THR_ADVERTISEMENT_I_MIN                             configurable value
            THR_ADVERTISEMENT_I_MAX                             configurable value
            THR_ADVERTISEMENT_K                                 configurable value
            THR_ROUTING_USE_LQI                                         0|1
            THR_NWK_PARTITIONING_ENABLE                                 0|1
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "stack_config.h"

/*==================================================================================================
Private macros
==================================================================================================*/

#ifndef THREAD_TASK_MSG_QUEUE_SIZE
    #define THREAD_TASK_MSG_QUEUE_SIZE 20
#endif

#ifndef THREAD_TASK_STACK_SIZE
   #define THREAD_TASK_STACK_SIZE 2048U
#endif

#ifndef THREAD_NODE_WAIT_FOR_PARENTS_TIMEOUT
    #define THREAD_NODE_WAIT_FOR_PARENTS_TIMEOUT 2000
#endif

#ifndef THREAD_NODE_WAIT_FOR_ROUTERS_UPDATE_TIMEOUT
    #define THREAD_NODE_WAIT_FOR_ROUTERS_UPDATE_TIMEOUT 2000
#endif

#ifndef THR_ROUTING_ENABLE
    #define THR_ROUTING_ENABLE 0
#endif

#ifndef THR_WEIGHTING_FACTOR
    #define THR_WEIGHTING_FACTOR 8
#endif

#ifndef THR_ID_REUSE_DELAY_SEC
    #define THR_ID_REUSE_DELAY_SEC 120
#endif

#ifndef THR_ROUTER_REMOVE_TIMEOUT_SEC
    #define THR_ROUTER_REMOVE_TIMEOUT_SEC 60
#endif

#ifndef THR_ID_SEQUENCE_PERIOD_SEC
    #define THR_ID_SEQUENCE_PERIOD_SEC 30
#endif

#ifndef THR_NETWORK_ID_TIMEOUT_SEC
    #define THR_NETWORK_ID_TIMEOUT_SEC 120
#endif

#ifndef THR_TIMER_PERIOD_SEC
    #define THR_TIMER_PERIOD_SEC 1
#endif

#ifndef THR_MAX_ROUTE_COST
    #define THR_MAX_ROUTE_COST  15
#endif

#ifndef THR_MAX_ROUTER_ID
    #define THR_MAX_ROUTER_ID  64

    /* update bit usage accordingly */   
    #define THR_ROUTER_BITS_SIZE 6
    #define THR_CHILD_BITS_SIZE 9   
#endif

#ifndef THR_LEASE_QUERRY_CACHE_TBL_SIZE 
    #define THR_LEASE_QUERRY_CACHE_TBL_SIZE  5
#endif

#ifndef THR_MAX_NEIGHBOR_AGE
    #define THR_MAX_NEIGHBOR_AGE 120
#endif

#ifndef THR_ADVERTISEMENT_I_MIN
    #define THR_ADVERTISEMENT_I_MIN  1
#endif

#ifndef THR_ADVERTISEMENT_I_MAX
    #define THR_ADVERTISEMENT_I_MAX   64
#endif

#ifndef THR_ADVERTISEMENT_K
    #define THR_ADVERTISEMENT_K   0xFF
#endif

#ifndef THREAD_TEST_FUNCTIONALITY
    #define THREAD_TEST_FUNCTIONALITY   1
#endif

#ifndef THR_ROUTING_USE_LQI
    #define THR_ROUTING_USE_LQI         0
#endif

#ifndef THR_NWK_PARTITIONING_ENABLE
    #define THR_NWK_PARTITIONING_ENABLE     0
#endif

#endif  /* THREAD_STACK_CFG_H */
