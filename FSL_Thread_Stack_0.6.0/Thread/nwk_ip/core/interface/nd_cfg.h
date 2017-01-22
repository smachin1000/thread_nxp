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

#ifndef _ND_CFG_H
#define _ND_CFG_H
/*!=================================================================================================
\file       nd_cfg.h
\brief      This is a configuration header file for the Neighbor Discovery for IP version 6 (IPv6)
            module.

\details    This file contains the folowing configuration options:

            ND_CONTEXT_SUPPORT_ENABLED            0 | 1 (default is 1)
            ND_NEIGHBOR_CACHE_SIZE               default is 5
            ND_PREFIX_LIST_SIZE                  default is 4
            ND_ROUTER_LIST_SIZE                  default is 2
            ND_DESTINATION_CACHE_SIZE            default is 4
            ND_DAD_TRANSMITS                     default is 1
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "stack_config.h"

/*==================================================================================================
Public macros
==================================================================================================*/

#ifndef ND_ENABLED
    #define ND_ENABLED 0
#endif

/*! Description of the ND_DAD_TRANSMITS configuration option: The number of consecutive Neighbor
    Solicitation messages sent while performing Duplicate Address Detection on a tentative address.
    A value of zero indicates that Duplicate Address Detection is not performed on tentative
    addresses. A value of one indicates a single transmission with no follow-up retransmissions. */
#ifndef ND_DAD_TRANSMITS
    #define ND_DAD_TRANSMITS (1U)
#endif

/*! Description of the ND_CLOSE_FEATURES_ENABLED configuration option: Enables/Disable closing of
    the ND session opened on a interface */
#ifndef ND_CLOSE_FEATURES_ENABLED
#define ND_CLOSE_FEATURES_ENABLED (0U)
#endif

/*! Description of the ND_MSG_RS_RCV_ENABLED configuration option: The device can/can`t receive and
    process a RS message */
#ifndef ND_MSG_RS_RCV_ENABLED
    #define ND_MSG_RS_RCV_ENABLED (1U)
#endif

/*! Description of the ND_LIFETIME_GUARD_INTERVAL configuration option: The amount of time in seconds
    before a context/prefix expires to send an RS */
#ifndef ND_LIFETIME_GUARD_INTERVAL
    #define ND_LIFETIME_GUARD_INTERVAL (10U)
#endif

/*! Description of the ND_REG_ADDRESS_GUARD_TIME configuration option: The amount of time in seconds
    before an address registration expires to send an NS with ARO */
#ifndef ND_REG_ADDRESS_GUARD_TIME
    #define ND_REG_ADDRESS_GUARD_TIME (10U)
#endif

/*! Description of the ND_MSG_DAC_RCV_ENABLED configuration option: The device can/can`t send a
    DAR message and process a DAC message */
#ifndef ND_MSG_DAC_RCV_ENABLED
    #define ND_MSG_DAC_RCV_ENABLED (0U)
#endif

/*! Description of the ND_MSG_DAR_RCV_ENABLED configuration option: The device can/can`t receive and
    process a RS message */
#ifndef ND_MSG_DAR_RCV_ENABLED
    #define ND_MSG_DAR_RCV_ENABLED (1U)
#endif

/*! Description of the ND_OPT_CONTEXT_RCV_ENABLED configuration option: The device can/can`t receive
    and process context options */
#ifndef ND_OPT_CONTEXT_RCV_ENABLED
    #define ND_OPT_CONTEXT_RCV_ENABLED (1U)
#endif

/*! Description of the ND_OPT_CONTEXT_SEND_ENABLED configuration option: The device can/can`t send
    context options */
#ifndef ND_OPT_CONTEXT_SEND_ENABLED
    #define ND_OPT_CONTEXT_SEND_ENABLED (1U)
#endif

/*! Description of the ND_OPT_AR_RCV_ENABLED configuration option: The device can/can`t receive and
    process an AR option */
#ifndef ND_OPT_AR_RCV_ENABLED
    #define ND_OPT_AR_RCV_ENABLED (1U)
#endif

/*! Description of the ND_OPT_AR_RCV_ENABLED configuration option: The device can/can`t use AR option
    to register itself to the router */
#ifndef ND_OPT_AR_SEND_ENABLED
    #define ND_OPT_AR_SEND_ENABLED (1U)
#endif

/*! Description of the ND_OPT_ABRO_RCV_ENABLED configuration option: The device can/can`t receive
    and process the ABR option */
#ifndef ND_OPT_ABRO_RCV_ENABLED
    #define ND_OPT_ABRO_RCV_ENABLED (1U)
#endif

/*! Description of the ND_CAN_DISABLE_DAD_ENABLED configuration option: The device can/can`t
    perform DAD on eui 64 base addresses */
#ifndef ND_CAN_DISABLE_DAD_ENABLED
    #define ND_CAN_DISABLE_DAD_ENABLED (1U)
#endif

/*! Description of the ND_RENEW_PREFIXES_ENABLED configuration option: The device can/can`t
    send RS messages to renew its prefixes before they expire */
#ifndef ND_RENEW_PREFIXES_ENABLED
    #define ND_RENEW_PREFIXES_ENABLED (0U)
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
#endif  /* _ND_CFG_H */
