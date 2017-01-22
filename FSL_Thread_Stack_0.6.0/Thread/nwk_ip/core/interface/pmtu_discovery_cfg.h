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

#ifndef  _PMTU_DISCOVERY_CFG_H
#define  _PMTU_DISCOVERY_CFG_H
/*!=================================================================================================
\file       pmtu_discovery_cfg.h
\brief      This is a configuration header file for the Path MTU Discovery module.

\details    This file contains the folowing configuration options:
            
            IGMP_V3_SOURCE_FILTERING_ENABLED        0 | 1   (default is 1)
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "stack_config.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/*! Description of the PMTU_PERIODIC_INCREASE_PMTU_ENABLED configuration option: set to 1 to enable 
    the periodical increase of PMTU in the Path MTU Discovery module */
#ifndef PMTU_PERIODIC_INCREASE_PMTU_ENABLED
    #define PMTU_PERIODIC_INCREASE_PMTU_ENABLED 1
#endif

/*! Description of the PMTU_CALLBACKS_ENABLED configuration option: set to 1 to enable 
    the possibility of registering callbacks that are called to notify the upper layers */
#ifndef PMTU_CALLBACKS_ENABLED
    #define PMTU_CALLBACKS_ENABLED 0
#endif

/*! Description of the PMTU_MANAGEMENT_FUNCTIONS_ENABLED configuration option: set to 1 to enable 
    the management functions of the Path MTU Discovery */
#ifndef PMTU_MANAGEMENT_FUNCTIONS_ENABLED
    #define PMTU_MANAGEMENT_FUNCTIONS_ENABLED 0
#endif

/*! Description of the PMTU_REGISTER_ICMP_PKT_TOO_BIG_HANDLER configuration option: set to 1 to  
    register the Packet Too Big Error Handler for packets send using ICMP */
#ifndef PMTU_REGISTER_ICMP_PKT_TOO_BIG_HANDLER
    #define PMTU_REGISTER_ICMP_PKT_TOO_BIG_HANDLER 1
#endif

/*! Description of the PMTU_REGISTER_TCP_PKT_TOO_BIG_HANDLER configuration option: set to 1 to  
    register the Packet Too Big Error Handler for packets send using TCP */
#ifndef PMTU_REGISTER_TCP_PKT_TOO_BIG_HANDLER
    #define PMTU_REGISTER_TCP_PKT_TOO_BIG_HANDLER 0
#endif

/*! Description of the PMTU_REGISTER_UDP_PKT_TOO_BIG_HANDLER configuration option: set to 1 to  
    register the Packet Too Big Error Handler for packets send using UDP */
#ifndef PMTU_REGISTER_UDP_PKT_TOO_BIG_HANDLER
    #define PMTU_REGISTER_UDP_PKT_TOO_BIG_HANDLER 0
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
#endif  /*  _PMTU_DISCOVERY_CFG_H */
