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

#ifndef  _ICMP_CFG_H
#define  _ICMP_CFG_H

/*!=================================================================================================
\file       icmp_cfg.h

\brief      This is a configuration header file for the ICMP module.

\details    This file contains the folowing configuration options:

            ICMP_STATISTICS_ENABLED                  0 | 1   (default is 1)
            ICMP_UNREGISTER_MSG_TYPE_HANDLER_ENABLED 0 | 1   (default is 1)
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "stack_config.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/*! Description of the ICMP_STATISTICS_ENABLED configuration option: set to 1 to enable statistics */
#ifndef ICMP_STATISTICS_ENABLED
#define ICMP_STATISTICS_ENABLED 0
#endif

/*! Description of the ICMP_UNREGISTER_MSG_TYPE_HANDLER_ENABLED configuration option: set to 1 to
enable unregister message type handler functionality */
#ifndef ICMP_UNREGISTER_MSG_TYPE_HANDLER_ENABLED
#define ICMP_UNREGISTER_MSG_TYPE_HANDLER_ENABLED 0
#endif

/*! Description of the ICMP_USE_ASYNC_SEND configuration option: If this option is enabled, the module
sends packets asynchronously using OS messages. This will save stack size but at the cost of code size
and slower operation. */
#ifndef ICMP_USE_ASYNC_SEND
#define ICMP_USE_ASYNC_SEND 0
#endif

/*!< Default hop limit value */
#ifndef ICMP_DEFAULT_HOP_LIMIT
#define ICMP_DEFAULT_HOP_LIMIT 128
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
