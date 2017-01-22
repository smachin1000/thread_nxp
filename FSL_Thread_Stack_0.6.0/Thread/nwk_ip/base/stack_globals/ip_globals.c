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
\file       ip_globals.c
\brief      This is the source file that contains parameters for the IP module that can be
            configured by the application
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "app_to_stack_config.h"
#include "ip.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/* Default Values */

#ifndef IP_IP6_ROUTING_TBL_SIZE
   #define IP_IP6_ROUTING_TBL_SIZE      15
#endif

#ifndef IP_IF_NB
   #define IP_IF_NB                     3
#endif

#ifndef IP_IF_IP6_ADDR_NB
   #define IP_IF_IP6_ADDR_NB            6
#endif

#ifndef IP_IF_IP6_MULTICAST_ADDR_NB
   #define IP_IF_IP6_MULTICAST_ADDR_NB  10
#endif

#ifndef IP_IF_IP4_ADDR_NB
   #define IP_IF_IP4_ADDR_NB            3
#endif

#ifndef IP_TRANSPORT_SERVICE_NB
   #define IP_TRANSPORT_SERVICE_NB      3
#endif

#ifndef IP_IP_REASSEMBLY_QUEUE_SIZE
    #define IP_IP_REASSEMBLY_QUEUE_SIZE 3
#endif

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

#if IP_IP6_ROUTING_ENABLE
/*! Routing Table */
ip6RoutingTblEntry_t *aIp6RoutingTable[IP_IP6_ROUTING_TBL_SIZE+1] = {NULL};
const uint32_t ip6RoutingTableSize = IP_IP6_ROUTING_TBL_SIZE;
#endif

/*! Interface Table */
ipIfStruct_t *aInterfaceTable[IP_IF_NB + 1] = {NULL};
const uint32_t interfaceTableSize = IP_IF_NB;

/*! IPv6 Global Address Table */
ip6IfAddrData_t* aGlobalAddrTable6[IP_IF_IP6_ADDR_NB+1] = {NULL};
const uint32_t globalAddrTable6Size = IP_IF_IP6_ADDR_NB;

/*! Multicast Address Table */
#if IP_IP6_ENABLE
ip6MulticastAddrData_t *aMulticastAddrTable[IP_IF_IP6_MULTICAST_ADDR_NB] = {NULL};
const uint32_t multicastAddrTableSize = IP_IF_IP6_MULTICAST_ADDR_NB;
#endif

/*! IPv4 Global Address Table */
#if IP_IP4_ENABLE
ip4IfAddrData_t* aGlobalAddrTable4[IP_IF_IP4_ADDR_NB] = {NULL};
const uint32_t globalAddrTable4Size = IP_IF_IP4_ADDR_NB;
#endif

/*! Transport Services List */
ipTransportServiceStruct_t *aTransportServiceList[IP_TRANSPORT_SERVICE_NB] = {NULL};
const uint32_t transportServiceListSize = IP_TRANSPORT_SERVICE_NB;

/*! IP Reassembly Queue Size */
const uint32_t ipReassemblyQueueSize = IP_IP_REASSEMBLY_QUEUE_SIZE;

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/

