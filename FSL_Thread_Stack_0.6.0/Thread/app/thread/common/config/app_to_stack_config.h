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

#ifndef _APP_TO_STACK_CONFIG_H_ 
#define _APP_TO_STACK_CONFIG_H_  

/*!=================================================================================================
\file       app_to_stack_config.h
\brief      This file is a for stack configuration of all thread demo applications.
            If it is required to configure just one application use the appllication config. file.
            Ex: for thread router application use thread_router_config.h 
==================================================================================================*/ 

#ifndef THREAD_DEFAULT_CAN_CREATE_NEW_NETWORK  
    #define THREAD_DEFAULT_CAN_CREATE_NEW_NETWORK       TRUE
#endif

#ifndef THREAD_DEFAULT_CAN_BECOME_ACTIVE_ROUTER  
    #define THREAD_DEFAULT_CAN_BECOME_ACTIVE_ROUTER     FALSE
#endif

#ifndef THREAD_DEFAULT_IS_POLLING_END_DEVICE   
    #define THREAD_DEFAULT_IS_POLLING_END_DEVICE        FALSE
#endif

#ifndef THREAD_DEFAULT_IS_GLOBAL_DHCP6_SERVER  
    #define THREAD_DEFAULT_IS_GLOBAL_DHCP6_SERVER       FALSE
#endif

#ifndef THREAD_DEFAULT_IS_BORDER_ROUTER  
    #define THREAD_DEFAULT_IS_BORDER_ROUTER             FALSE 
#endif

#ifndef THREAD_USE_SHELL  
    #define THREAD_USE_SHELL          TRUE
#endif   

#ifndef THREAD_USE_FSCI  
    #define THREAD_USE_FSCI           FALSE    
#endif 
   
#if (THREAD_DEFAULT_CAN_CREATE_NEW_NETWORK && THREAD_DEFAULT_IS_POLLING_END_DEVICE) || \
    (THREAD_DEFAULT_CAN_BECOME_ACTIVE_ROUTER && THREAD_DEFAULT_IS_POLLING_END_DEVICE)
    #error There are included two not compatible configurations. Please select only one configuration
#endif

#if (THREAD_USE_SHELL && THREAD_USE_FSCI)
    #error There are included two not compatible methods. Please select only one method.
#endif

/*!=================================================================================================
   Stack Globals Configuration 
==================================================================================================*/ 

/*!=================================================================================================
  THREAD 
==================================================================================================*/ 

/*! The maximum number of Thread Interfaces. MUST not be greater that IP_IF_NB */
#ifndef THR_MAX_INTERFACES
    #define THR_MAX_INTERFACES                  1
#endif

/*! The maximum number of allowed Routers in the Thread network */
#ifndef THR_MAX_ALLOWED_ROUTERS
    #define THR_MAX_ALLOWED_ROUTERS             32
#endif

/*! The maximum number of radio range neighbors with which the Thread device can communicate with 
    MUST be equal to gNumKeyDeviceListEntries_c macro found in the MAC configuration */
#ifndef THREAD_MLE_MAX_NEIGHBORS
    #define THREAD_MLE_MAX_NEIGHBORS            60
#endif

/*! The number of cache entries a Thread device can maintain */
#ifndef THR_LEASE_QUERRY_CACHE_TBL_SIZE
    #define THR_LEASE_QUERRY_CACHE_TBL_SIZE     15
#endif

/*! Sleepy End Device data fast polling rate. Used for example when soliciting a global address via 
DHCP Solicit */
#ifndef THREAD_FAST_POLLING_INTERVAL
    #define THREAD_FAST_POLLING_INTERVAL        100      /* Miliseconds */
#endif
/*!=================================================================================================
   DHCPv6 
==================================================================================================*/

/*! The maximum number of DHCPv6 servers that can be started on the device */
#ifndef DHCP6_SERVER_MAX_INSTANCES
    #define DHCP6_SERVER_MAX_INSTANCES          2
#endif

/*! The maximum number of DHCPv6 clients that the device can service as a DHCPv6 server */
#ifndef DHCP6_SERVER_MAX_CLIENTS
    #define DHCP6_SERVER_MAX_CLIENTS            100
#endif

/*! The maximum number of DHCPv6 clients that can be started on the device */
#ifndef DHCP6_CLIENT_MAX_INSTANCES
    #define DHCP6_CLIENT_MAX_INSTANCES          5
#endif

/*!=================================================================================================
   COAP 
==================================================================================================*/

/*! The maximum number of COAP sessions that can be established at one time */
#ifndef COAP_MAX_SESSIONS
    #define COAP_MAX_SESSIONS                   5
#endif

/*!=================================================================================================
   MLE 
==================================================================================================*/

/*! Macro that defines how many security keys the MLE module can store at one time */
#ifndef MLE_KEY_DESCRIPTOR_TABLE_SIZE
    #define MLE_KEY_DESCRIPTOR_TABLE_SIZE       2
#endif



/*!=================================================================================================
   SOCKETS 
==================================================================================================*/

/*! The maximum number of sockets that can be opened at one time. MUST be corelated to MAX_UDP_CONNECTIONS */
#ifndef BSDS_MAX_SOCKETS
    #define BSDS_MAX_SOCKETS                    30
#endif

/*!=================================================================================================
   UDP 
==================================================================================================*/

/*! The maximum number of UDP connections that can be opened at one time. MUST not be greater than BSDS_MAX_SOCKETS */
#ifndef MAX_UDP_CONNECTIONS
    #define MAX_UDP_CONNECTIONS                 30
#endif

/*!=================================================================================================
   IP
==================================================================================================*/

/*! The maximum number of IP route entries */
#ifndef IP_IP6_ROUTING_TBL_SIZE
    #define IP_IP6_ROUTING_TBL_SIZE             6
#endif

/*! The maximum supported number of IP interfaces */
#ifndef IP_IF_NB
    #define IP_IF_NB                            1
#endif

/*! The maximum number of IPv6 addresses. This is regardless of how many interfaces are available */
#ifndef IP_IF_IP6_ADDR_NB
    #define IP_IF_IP6_ADDR_NB                   8
#endif

/*! The maximum number of supported multicast addresses */
#ifndef IP_IF_IP6_MULTICAST_ADDR_NB
    #define IP_IF_IP6_MULTICAST_ADDR_NB         10
#endif

/*! The maximum number of IP transport services that can be supported. Ex. UDP, TCP. */
#ifndef IP_TRANSPORT_SERVICE_NB
    #define IP_TRANSPORT_SERVICE_NB             3
#endif

/*! Number representing how many IP packet fragments can be stored at one time */
#ifndef IP_IP_REASSEMBLY_QUEUE_SIZE
    #define IP_IP_REASSEMBLY_QUEUE_SIZE         3
#endif

/*!=================================================================================================
   MPL 
==================================================================================================*/

/*! The maximum number of MPL instances. This must be correlated to IP_IF_NB. */
#ifndef MPL_INSTANCE_SET_SIZE
    #define MPL_INSTANCE_SET_SIZE               2
#endif

/*! The maximum number of seeds the MPL module can store at one time */
#ifndef MPL_SEED_SET_SIZE
    #define MPL_SEED_SET_SIZE                   5
#endif

/*! The maximum number of MPL transmited messages that can be buffered at one time */
#ifndef MPL_BUFFERED_MESSAGE_SET_SIZE
    #define MPL_BUFFERED_MESSAGE_SET_SIZE       5
#endif

/*!=================================================================================================
   TRICKLE 
==================================================================================================*/
/*! The maximum number of TRICKLE instances. This must be correlated to IP_IF_NB. */
#ifndef TRICKLE_INSTANCE_SET_SIZE
    #define TRICKLE_INSTANCE_SET_SIZE           2
#endif

/*! The maximum number of Trickle events */
#ifndef TRICKLE_LIST_SIZE
    #define TRICKLE_LIST_SIZE                   5
#endif

/*!=================================================================================================
   SIXLOWPAN 
==================================================================================================*/

/*! The maximum number of 6LoWPAN instaces. MUST not be greater than IP_IF_NB. */
#ifndef SLWPCFG_INSTANCES_NB
    #define SLWPCFG_INSTANCES_NB                1
#endif

/*! The maximum number of 6LoWPAN contexts that can be stored */
#ifndef SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE
    #define SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE  16
#endif

/*! Enables/Disables the usage of secured Data Polling requests in the case of Sleepy End Devices */
#ifndef MAC_SEC_POLL_ENABLED
    #define MAC_SEC_POLL_ENABLED                1
#endif    

/*!=================================================================================================
   MAC 
==================================================================================================*/
/*! Enables/Disables the MAC Filtering number */
#ifndef MAC_FILTERING_ENABLED
  #define MAC_FILTERING_ENABLED                 0
#endif

/*! The maximum number of entries in the MAC filtering table */
#ifndef MAC_FILTERING_TABLE_SIZE
    #define MAC_FILTERING_TABLE_SIZE            10
#endif

/*!=================================================================================================
  EVENT MANAGER 
==================================================================================================*/

/*! The maximum number of entries in the Event Manager table */
#ifndef EVM_TABLE_SIZE
    #define EVM_TABLE_SIZE                      15
#endif

         
#endif /* _APP_TO_STACK_CONFIG_H_ */
