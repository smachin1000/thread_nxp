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

#ifndef _MDNS_H
#define _MDNS_H
/*!=================================================================================================
\file       dns_client.h
\brief      This is a header file for the dns client module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "sockets.h"
/*==================================================================================================
Public macros
==================================================================================================*/
#define     MDNS_CACHE_FLUSH_MASK       (0x8000U)
#define     MDNS_UNICAST_RESPONSE_MASK  (MDNS_CACHE_FLUSH_MASK)
#define     MDNS_FIRST_ANNOUNCING_STEP  (4U)

/*
Prefixes go a step beyond the base type and describe the use of a variable. Following prefixes shall
exists
        g   // global variable
        m   // module variable
        c   // count (as in the number of records, characters, and so on)
        p   // pointer
        i   // index into an array
        a   // array
        e   // element of array
        h   // head, e.g. for a head index into an array the prefix becomes ih
        t   // tail, e.g. for a tail pointer the prefix becomes pt
        s   // string
        f   // function, e.g. for a pointer to a function the prefix is pf
        d   // a conditional define (not a variable, but uses the notation)
*/

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum
{
    mMdnsProbingOK_c            = 0,
    mMdnsProbingTimerInvalid_c  = 1,
    mMdnsProbingNoMemory_c      = 2

} MdnsProbingResponse_t;

typedef struct mDnsService_tag
{
    uint8_t *name;              /*! host name */
    uint8_t *txtInfo;           /*! service information */
    uint8_t *domain;            /*! usually local */
    uint8_t *protocolType;      /*! e.g.: _http._tcp */
    uint8_t *fullName;          /*! host._http._tcp.local */
    ipAddr_t *address;          /*! ip address of the service */
    uint32_t portNo;            /*! service port */
    bool_t isAdvertised;

} mDnsService_t;

typedef struct mDnsProbingStruct_tag
{
    mDnsService_t *newService;
    uint8_t *pData;
    tmrTimerID_t probingTmrId;
    uint8_t counter;
    uint16_t dataLength;
} mDnsProbingStruct_t;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/

uint8_t MDNS_probeNewService(mDnsService_t *newService);
void MDNS_JoinIf(ifHandle_t* pIfHandle);
void MDNS_Init(taskMsgQueue_t *pTaskMsgQueue);
void MDNS_service(void *pInData);




#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _MDNS_H */
