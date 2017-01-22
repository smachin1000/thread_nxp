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

#ifndef _SOCKETS_CFG_H
#define _SOCKETS_CFG_H
/*!=================================================================================================
\file       sockets_cfg.h
\brief      This is a header file for the Sockets module. It contains the configuration file for the
            sockets implementation.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "stack_config.h"

/*==================================================================================================
Public macros
==================================================================================================*/
#ifndef BSDS_CHECK_ADDRSIZE
#   define BSDS_CHECK_ADDRSIZE      (1)     /*!< Verify socket address size */
#endif
#ifndef BSDS_DATAGRAM_SUPPORT
#   define BSDS_DATAGRAM_SUPPORT    (1)     /*!< Support datagram sockets(using UDP) */
#endif
#ifndef BSDS_STREAM_SUPPORT
#   define BSDS_STREAM_SUPPORT      (0)     /*!< Support stream sockets(using TCP) */
#endif
#ifndef BSDS_SELECT_SUPPORT
#   define BSDS_SELECT_SUPPORT      (1)     /*!< Sockets module support select functionality */
#endif
#ifndef BSDS_OPTIONS_SUPPORT
#   define BSDS_OPTIONS_SUPPORT     (1)     /*!< Support socket options */
#endif
#ifndef BSDS_SELECT_MAX_FDS
#   define BSDS_SELECT_MAX_FDS      (20)    /*!< Maximum number of file descriptors to be added */
#endif

/* Checks */
#if (TCP_ENABLED) && (!BSDS_STREAM_SUPPORT)
#   undef BSDS_STREAM_SUPPORT
#   define BSDS_STREAM_SUPPORT      (1)
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
#endif  /* _SOCKETS_CFG_H */
