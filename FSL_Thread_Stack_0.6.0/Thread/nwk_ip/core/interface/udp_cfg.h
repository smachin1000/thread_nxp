
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

#ifndef _UDP_CFG_H
#define _UDP_CFG_H
/*!=================================================================================================
\file       udp_cfg.h
\brief      This is a header file for the UDP module. It contains macros used to configure the
            UDP module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "stack_config.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#ifndef UDP_MIN_PORT_NUMBER
#   define UDP_MIN_PORT_NUMBER      (1024U) /*!< minimum port number available for users(1024) */
#endif
#ifndef UDP_MAX_PORT_NUMBER
#define UDP_MAX_PORT_NUMBER         (4096U) /*!< maximum port number available for users(49151) */
#endif
#ifndef UDP_MAX_RX_PACKETS
#   define UDP_MAX_RX_PACKETS       (5U)    /*!< maximum number of packets to be in the RX queue */
#endif
#ifndef UDP_TREAT_ICMP_PORT_UNR
#   define UDP_TREAT_ICMP_PORT_UNR  (1)     /*!< treat ICMP Port Unreachable error */
#endif
#ifndef UDP_TREAT_CHECKSUM
#   define UDP_TREAT_CHECKSUM       (1)     /*!< Enable/Disable checksum calculation for UDP header */
#endif
#ifndef UDP_DEBUG
#   define UDP_DEBUG                (0)
#endif
#ifndef UDP_DEFAULT_HOP_LIMIT
#   define UD_DEFAULT_HOP_LIMIT    (128)   /*!< Default hop limit for IPv6 or TTL for IPv4 */
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
#endif  /* _UDP_CFG_H */
