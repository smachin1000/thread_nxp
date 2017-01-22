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


#ifndef _DHCP_CFG_H
#define _DHCP_CFG_H
/*!=================================================================================================
\file       dhcp_cfg.h
\brief      This is a header file for a configuration header file. It contains default values for
            module configuration defines.
\details    This file contains the folowing configuration options:

            DHCP4_SERVER_ENABLED                0 | 1
            DHCP4_CLIENT_ENABLED                0 | 1
            DHCP_TIMER_PERIOD_MS           default is one hour (0x36EE80)
            DHCP_TIMER_PERIOD_SEC          default id one hour (0xE10)
            DHCP_SERVER_MAX_CLIENTS        default is 5U
            DHCP_CLIENT_REQ_TIMER          default is 10000U
            DHCP_CLIENT_DEF_RETRY_NB       default is 4U
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "stack_config.h"

/*==================================================================================================
Private macros
==================================================================================================*/

#ifndef DHCP4_SERVER_ENABLED
    #define DHCP4_SERVER_ENABLED            0
#endif

#ifndef DHCP4_CLIENT_ENABLED
    #define DHCP4_CLIENT_ENABLED            0
#endif

/*! timer period */
#ifndef DHCP_TIMER_PERIOD_MS
    #define DHCP_TIMER_PERIOD_MS            0x36EE80 /* one hour in miliseconds*/ //10000U//
#endif

#ifndef DHCP_TIMER_PERIOD_SEC
    #define DHCP_TIMER_PERIOD_SEC           0xE10    /* one hour in seconds */ //10
#endif

/*! timer period */
#ifndef DHCP_CLIENT_REQ_TIMER
    #define DHCP_CLIENT_REQ_TIMER           10000U
#endif

/*! number of retries*/
#define DHCP_CLIENT_DEF_RETRY_NB 4U

#endif  /*_DHCP_CFG_H */
