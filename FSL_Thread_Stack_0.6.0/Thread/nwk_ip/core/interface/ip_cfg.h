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

#ifndef _IP_CFG_H
#define _IP_CFG_H

/*!=================================================================================================
\file       ip_cfg.h
\brief      This is a configuration header file for IP layer module. It contains default values for
            module configuration defines.
\details    This file contains the folowing configuration options:

            IP_IP6_ENABLE                         0|1
            IP_IP4_ENABLE                         0|1
            IP_IP6_ROUTING_TBL_SIZE        configurable value
            IP_IF_IP6_ADDR_NB              configurable value
            IP_IF_IP6_MULTICAST_ADDR_NB    configurable value
            IP_IF_IP4_ADDR_NB              configurable value
            IP_TRANSPORT_SERVICE_NB        configurable value
            IP_IF_NB                       configurable value
            IP_TASK_MSG_QUEUE_SIZE         configurable value
            IP_IP_REASSEMBLY_QUEUE_SIZE    configurable value
            IP_IF_MAC_ADDR_NB              configurable value
            IP_IP6_STATS_ENABLE                   0|1
            IP_IP4_STATS_ENABLE                   0|1
            IP_IF_STATS_ENABLE                    0|1
            IP_IP6_ROUTING_ENABLE                 0|1
            IP_IP6_ENABLE_FRAG                    0|1
            IP_IP4_ENABLE_FRAG                    0|1
            IP_IP6_ENABLE_REASSEMBLY              0|1
            IP_IP4_ENABLE_REASSEMBLY              0|1
            IP_IP6_LOOPBACK_MULTICAST             0|1
            IP_IP6_LOOPBACK                       0|1
            IP_DISCARD_SELF_BCASTS                0|1
            IP_DISABLE_INTERFACE_FUNCTIONALITY    0|1
            IP_USE_ASYNC_SEND                     0|1
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "stack_config.h"

/*==================================================================================================
Private macros
==================================================================================================*/

#ifndef IP_IP6_ENABLE
   #define IP_IP6_ENABLE 0
#endif

#ifndef IP_IP4_ENABLE
   #define IP_IP4_ENABLE 0
#endif

#if (IP_IP6_ENABLE == 0) && (IP_IP4_ENABLE == 0)
#error "Cannot disable both IPv6 and IPv4"
#endif

#ifndef IP_TASK_STACK_SIZE
   #define IP_TASK_STACK_SIZE 1024U
#endif

#ifndef IP_TASK_MSG_QUEUE_SIZE
   #define IP_TASK_MSG_QUEUE_SIZE 20U
#endif

#ifndef IP_IF_MAC_ADDR_NB
   #define IP_IF_MAC_ADDR_NB 2U
#endif

#ifndef IP_IP6_STATS_ENABLE
    #define IP_IP6_STATS_ENABLE 0
#endif

#ifndef IP_IP4_STATS_ENABLE
    #define IP_IP4_STATS_ENABLE 0
#endif

#ifndef IP_IF_STATS_ENABLE
    #define IP_IF_STATS_ENABLE 0
#endif

#ifndef IP_IP6_ROUTING_ENABLE
    #define IP_IP6_ROUTING_ENABLE 0
#endif

#ifndef IP_IP6_ENABLE_FRAG
    #define IP_IP6_ENABLE_FRAG 1
#endif

#ifndef IP_IP4_ENABLE_FRAG
    #define IP_IP4_ENABLE_FRAG 0
#endif

#ifndef IP_IP6_ENABLE_REASSEMBLY
    #define IP_IP6_ENABLE_REASSEMBLY 1
#endif

#ifndef IP_IP4_ENABLE_REASSEMBLY
    #define IP_IP4_ENABLE_REASSEMBLY 0
#endif

#ifndef IP_IP6_LOOPBACK_MULTICAST
    #define IP_IP6_LOOPBACK_MULTICAST 0
#endif

#ifndef IP_IP6_LOOPBACK
    #define IP_IP6_LOOPBACK 0
#endif

#ifndef IP_DISCARD_SELF_BCASTS
    #define IP_DISCARD_SELF_BCASTS 0
#endif

#ifndef IP_DISABLE_INTERFACE_FUNCTIONALITY
    #define IP_DISABLE_INTERFACE_FUNCTIONALITY 0
#endif

#ifndef IP_USE_ASYNC_SEND
    #define IP_USE_ASYNC_SEND 1
#endif

#endif  /*_IP_CFG_H */
