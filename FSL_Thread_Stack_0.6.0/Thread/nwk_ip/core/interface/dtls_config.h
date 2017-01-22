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


#ifndef _DTLS_CONFIG_H
#define _DTLS_CONFIG_H
/*!=================================================================================================
\file       dtls_config.h
\brief      Dtls Configuration header.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/*==================================================================================================
Public macros
==================================================================================================*/
#ifndef DTLS_ENABLED
#   define DTLS_ENABLED                 (0U)
#endif

#ifndef DTLS_MAX_CONTEXTS
#   define DTLS_MAX_CONTEXTS            (1U)
#endif

#ifndef DTLS_MAX_PEERS
#   define DTLS_MAX_PEERS               (3U)
#endif

#ifndef DTLS_SESSION_ID_LENGTH
#   define DTLS_SESSION_ID_LENGTH       (32U)
#endif

#ifndef DTLS_COOKIE_LENGTH
    #define DTLS_COOKIE_LENGTH          (8U)
#endif


#ifndef DTLS_RETRANSMIT_ENABLED
#   define DTLS_RETRANSMIT_ENABLED       (1)
#endif

/*!< The retransmission timeout is defined as being the number of retransmission units
 *   in milliseconds.   */
#ifndef DTLS_RETRANSMIT_UNIT
#   define DTLS_RETRANSMIT_UNIT          (100U) /* ms*/
#endif


/*!< Thread Interop 3. This skips first ClientHello and HelloVerifyRequest messages for Thread
 * Interop 3 purposes */
#ifndef DTLS_THREAD_INTEROP3
#   define DTLS_THREAD_INTEROP3          (0)
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

/*================================================================================================*/
#endif
