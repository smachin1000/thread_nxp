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
#ifndef _DEBUG_LOG_H
#define _DEBUG_LOG_H
/*!=================================================================================================
\file       debug_log.h
\brief      This is the header file for the module to print debug messages.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include <stddef.h>
#include <stdarg.h>


/*==================================================================================================
Public macros
==================================================================================================*/
#ifndef DEBUG_LOG
#define DEBUG_LOG    1
#endif

#if DEBUG_LOG
#if THREAD_USE_SHELL
    #undef SHELL_USE_PRINTF
    #define SHELL_USE_PRINTF 1
    //#include "shell.h"
    extern uint16_t shell_printf(char * format,...);
    extern void shell_write(char *pBuff);
    extern void shell_writeN(char *pBuff, uint16_t n);
#else
    #define shell_printf(a, ...)
    #define shell_write(a)
    #define shell_writeN(b)
#endif    
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
#if DEBUG_LOG
    void DBG_PrintLog( bool_t printTimeStamp, uint8_t* pString,
                       uint8_t* pBuf, size_t buflen);
    void DBG_PrintTimeStamp(void);
    void DBG_HexDump(uint8_t *pBuf, uint16_t buflen);
    #define DBG_Printf(format,...)    shell_printf(format, __VA_ARGS__)
    #define DBG_WriteString(pBuf) shell_write(pBuf)
    #define DBG_PrintNBytes(pBuff, n)  shell_writeN(pBuff, n)
    void    DBG_MEMBufferCheck(uint8_t *p, uint32_t size);
    void    DBG_MsgCheck(void);
#else
    #define DBG_PrintLog(printTimeStamp, pString, len, pBuf)
    #define DBG_PrintTimeStamp()
    #define DBG_HexDump(pBuf,buflen)
    #define DBG_Printf(format,...)
    #define DBG_WriteString(pBuf)
    #define DBG_PrintNBytes(pBuff, n)
    #define DBG_MEMBufferCheck(p, size)
    #define DBG_MsgCheck()
#endif
/*TODO: Add more functions here */

/*================================================================================================*/
#endif
