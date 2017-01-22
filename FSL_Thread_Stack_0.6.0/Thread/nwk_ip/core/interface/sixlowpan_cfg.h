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

#ifndef  _SIXLOWPAN_CFG_H
#define  _SIXLOWPAN_CFG_H
/*!=================================================================================================
\file       sixlowpan_cfg.h
\brief      This is a configuration header file for the 6LoWPAN module.
\details    This file contains the folowing configuration options:

            ICMP_ENABLE_ICMPV6_SUPPORT                      0 | 1   (default is 1)

            SLWPCFG_INSTANCES_NB                            0 | 1   (default is 1)

            SLWPCFG_STACK_SIZE                              Default is 768U

            SLWPCFG_MSG_QUEUE_SIZE                          Default is 10

            SLWPCFG_SERVER_ENABLED                          0 | 1   (default is 1)

            SLWPCFG_NODE_ENABLED                            0 | 1   (default is 1)

            SLWPCFG_MESH_ENABLED                            0 | 1   (default is 1)

            SLWPCFG_FRAGMENTATION_ENABLED                   0 | 1   (default is 1)

            SLWPCFG_HEADER_COMPRESSION_ENABLED              0 | 1   (default is 1)

            SLWPCFG_RFC6282_COMPRESSION_ENABLED             0 | 1   (default is 1)

            SLWPCFG_RFC4944_COMPRESSION_ENABLED             0 | 1   (default is 0)

            SLWPCFG_BSTRAP_ENABLED                          0 | 1   (default is 0)

            SLWPCFG_LBP_ENABLED                             0 | 1   (default is 1)

            SLWPCFG_ROUTING_ENABLED                         0 | 1   (default is 0)

            SLWPCFG_LOADNG_ENABLED                          0 | 1   (default is 0)

            SLWPCFG_MAX_NSDU_LENGTH                         Default is 1280U

            SLWPCFG_IB_MAX_HOPS                             Default is 10U

            SLWPCFG_IB_GROUP_TABLE_MAX_ELEMENTS             Default is 4

            SLWPCFG_RFC4944_FRAG_TIMEOUT_MS                 Default is 1500 ms

            SLWPCFG_RFC4944_FRAG_QUEUE_SIZE                 Default is 5

            SLWPCFG_RFC4944_TX_PACKET_CLEANUP_ENABLED       0 | 1   (default is 0)

            SLWPCFG_RFC4944_TX_PACKET_TIMEOUT               Default is 1 sec

            SLWPCFG_RFC4944_MAX_FRAGMENTS                   Default is 20

            SLWPCFG_RFC6282_COMPRESS_UDP_CHECKSUM           0 | 1   (default is 1)
              
            SLWPCFG_RFC6282_REMOVE_PAD                      0 | 1   (default is 1)

            SLWPCFG_RFC6282_CONTEXT_TABLE_SIZE              Default is 2
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "stack_config.h"

/*==================================================================================================
Public macros
==================================================================================================*/

#ifndef SLWPCFG_STACK_SIZE
   #define SLWPCFG_STACK_SIZE  768U
#endif

#ifndef SLWPCFG_MSG_QUEUE_SIZE
   #define SLWPCFG_MSG_QUEUE_SIZE  10
#endif

#ifndef SLWPCFG_SERVER_ENABLED
    #define SLWPCFG_SERVER_ENABLED 1
#endif

#ifndef SLWPCFG_NODE_ENABLED
    #define SLWPCFG_NODE_ENABLED 1
#endif

#ifndef SLWPCFG_MESH_ENABLED
   #define SLWPCFG_MESH_ENABLED 1
#endif

#ifndef SLWPCFG_FRAGMENTATION_ENABLED
   #define SLWPCFG_FRAGMENTATION_ENABLED 1
#endif

#ifndef SLWPCFG_HEADER_COMPRESSION_ENABLED
   #define SLWPCFG_HEADER_COMPRESSION_ENABLED 1
#endif

#if (SLWPCFG_HEADER_COMPRESSION_ENABLED)
    #ifndef SLWPCFG_RFC6282_COMPRESSION_ENABLED
        #define SLWPCFG_RFC6282_COMPRESSION_ENABLED 1
    #endif

    #ifndef SLWPCFG_RFC4944_COMPRESSION_ENABLED
        #define SLWPCFG_RFC4944_COMPRESSION_ENABLED 1
    #endif
#else
    #define SLWPCFG_RFC4944_COMPRESSION_ENABLED 0
    #define SLWPCFG_RFC6282_COMPRESSION_ENABLED 0
#endif

#ifndef SLWPCFG_BSTRAP_ENABLED
    #define SLWPCFG_BSTRAP_ENABLED  0
#endif

#if (SLWPCFG_BSTRAP_ENABLED)
    #ifndef SLWPCFG_LBP_ENABLED
        #define SLWPCFG_LBP_ENABLED 1
    #endif
#else
    #define SLWPCFG_LBP_ENABLED 0
#endif

#ifndef SLWPCFG_ROUTING_ENABLED
    #define SLWPCFG_ROUTING_ENABLED 0
#endif

#if (SLWPCFG_ROUTING_ENABLED)
    #ifndef SLWPCFG_LOADNG_ENABLED
        #define SLWPCFG_LOADNG_ENABLED 0
    #endif
#else
    #define SLWPCFG_LOADNG_ENABLED 0
#endif

/*! Maximum length of a data frame trasmitted over 6LoWPAN */
#ifndef SLWPCFG_MAX_NSDU_LENGTH
    #define SLWPCFG_MAX_NSDU_LENGTH 1280U
#endif

#ifndef SLWPCFG_MAX_6LOWPAN_RETRANSMISSIONS
    #define SLWPCFG_MAX_6LOWPAN_RETRANSMISSIONS 1
#endif

/*! Maximum number of hops */
#ifndef SLWPCFG_IB_MAX_HOPS
    #define SLWPCFG_IB_MAX_HOPS 10U
#endif

#ifndef SLWPCFG_IB_GROUP_TABLE_MAX_ELEMENTS
    #define SLWPCFG_IB_GROUP_TABLE_MAX_ELEMENTS 4
#endif

#ifndef SLWPCFG_RFC4944_FRAG_TIMEOUT_MS
    #define SLWPCFG_RFC4944_FRAG_TIMEOUT_MS 1500
#endif

#ifndef SLWPCFG_RFC4944_FRAG_QUEUE_SIZE
    #define SLWPCFG_RFC4944_FRAG_QUEUE_SIZE 5
#endif

#ifndef SLWPCFG_RFC4944_TX_PACKET_CLEANUP_ENABLED
    #define SLWPCFG_RFC4944_TX_PACKET_CLEANUP_ENABLED 0
#endif

#ifndef SLWPCFG_RFC4944_TX_PACKET_TIMEOUT
    #define SLWPCFG_RFC4944_TX_PACKET_TIMEOUT 1
#endif

#ifndef SLWPCFG_RFC4944_MAX_FRAGMENTS
    #define SLWPCFG_RFC4944_MAX_FRAGMENTS 20
#endif

#ifndef SLWPCFG_RFC6282_COMPRESS_UDP_CHECKSUM
    #define SLWPCFG_RFC6282_COMPRESS_UDP_CHECKSUM 0
#endif

#ifndef SLWPCFG_RFC6282_REMOVE_PAD
    #define SLWPCFG_RFC6282_REMOVE_PAD 0
#endif

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


#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*  _ICMP_CFG_H */
