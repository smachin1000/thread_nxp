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

#ifndef _TCP_CFG_H
#define _TCP_CFG_H
/*!=================================================================================================
\file       tcp_cfg.h
\brief      This is a header file for the TCP module. It contains macros used to configure the
            TCP module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "stack_config.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#ifndef TCP_ENABLED
#   define TCP_ENABLED              (0)     /*!< Enable/disable module */
#endif
#ifndef TCP_MIN_PORT_NUMBER
#   define TCP_MIN_PORT_NUMBER      (1024U) /*!< minimum port number available for users(1024) */
#endif
#ifndef TCP_MAX_PORT_NUMBER
#define TCP_MAX_PORT_NUMBER         (4096U) /*!< maximum port number available for users(49151) */
#endif
#ifndef TCP_MAX_RX_PACKETS
#   define TCP_MAX_RX_PACKETS       (4U)   /*!< maximum number of packets to be kept in the RX queue */
#endif
#ifndef TCP_TREAT_ICMP_PORT_UNR
#   define TCP_TREAT_ICMP_PORT_UNR  (1)     /*!< treat ICMP Port Unreachable error */
#endif

/* TCP configurations */
#ifndef TCP_DEBUG
#   define TCP_DEBUG                (0)     /*!< TCP debugging messages */
#endif
#ifndef TCP_RCV_WND_MANAGEMENT
#   define TCP_RCV_WND_MANAGEMENT   (0)     /*!< Enable window management */
#endif
#ifndef TCP_HANDLE_FULL_QUEUE
#   define TCP_HANDLE_FULL_QUEUE    (0)     /*!< When the RX queue is full block the TCP(IP) Task
                                             with osSignalWait and when one element is extracted
                                             from the queue unblock the TCP(IP) task with
                                             osSignalSet and add the waiting packet */
#endif

#ifndef TCP_CONGESTION
#   define TCP_CONGESTION           (0)     /*!< "Congestion Avoidance and Control," V. Jacobson,
                                            ACM SIGCOMM-88, August 1988 */
#endif

#ifndef TCP_STATS_ENABLE
#   define TCP_STATS_ENABLE         (1)
#endif

#ifndef TCP_OPTIONS_ENABLE
#   define TCP_OPTIONS_ENABLE       (1)
#endif

#ifndef TCP_REUSEADDR
#   define TCP_REUSEADDR            (1)
#endif

#ifndef TCP_DEFAULT_HOP_LIMIT
#   define TCP_DEFAULT_HOP_LIMIT    (128)   /*!< Default hop limit for IPv6 or TTL for IPv4 */
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
#endif  /* _TCP_CFG_H */
