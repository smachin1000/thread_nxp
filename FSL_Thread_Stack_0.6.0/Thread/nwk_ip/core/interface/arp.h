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


#ifndef _ARP_H_
#define _ARP_H_
/*!=================================================================================================
\file       arp.h
\brief      This is a header file for the ARP module. 
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "ip.h"             

#if IP_IP4_ENABLE
/*==================================================================================================
Public macros
==================================================================================================*/
#ifndef ARP_CACHE_SIZE
   #define ARP_CACHE_SIZE      (5U) 
#endif

#ifndef ARP_MAX_LIFETIME
    #define ARP_MAX_LIFETIME   (1200U)
#endif

#ifndef ARP_PENDING_LIFETIME
    #define ARP_PENDING_LIFETIME (5U)
#endif

#ifndef ARP_UNREACHABLE_LIFETIME
    #define ARP_UNREACHABLE_LIFETIME     (20U)
#endif

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum
{
    gArpCodeRequest_c =0X0001U,
    gArpCodeReply_c =0x0002U
}arpCode_t;


typedef enum 
{
    gArpStateEmpty_c    = 0U,
    gArpStateResolved_c,
    gArpStatePending_c,
    gArpStateUnreachable
} arpState_t;

typedef struct arpPacket_tag
{
    uint8_t linkLayerType[2];
    uint8_t protoType[2];
    uint8_t linkLayerLength;
    uint8_t protoLength;
    uint8_t opCode[2];
    uint8_t senderMACAddr[6];
    uint8_t senderIPaddr[4];
    uint8_t targetMACaddr[6];
    uint8_t targetIPaddr[4];
} arpPacket_t;



typedef struct arpCacheEntry_tag
{
    uint32_t  ipAddress;
    ifHandle_t* IfHandle; 
    uint32_t lifetime;
    ipPktInfo_t *pWaitingPacket;
    llAddr_t llAdress;
    uint8_t state;
} arpCacheEntry_t;


/*! Function for interfacing ARP module with LL Driver Send */
typedef void (* arpSend)(uint8_t*, uint32_t, llAddr_t*);

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn void ARP_Service(void* pInData)
                                                   
\brief  Parses a received ARP packet

\param [in]   pInData  Pointer to the received packet.
                             
\retval       none
***************************************************************************************************/
void ARP_Service(void* pInData);

/*!*************************************************************************************************
\fn llAddr_t* void ARP_Resolve(ipPktInfo_t* pIpPktInfo)
                                                   
\brief This function finds the matching link layer address for the given IP address or sends an
ARP packet request if it is not found.  

\param [in]   pIpPktInfo   Pointer to the IP packet.
                             
\retval       none
***************************************************************************************************/
llAddr_t* ARP_Resolve(ipPktInfo_t* pIpPktInfo);

/*!*************************************************************************************************
\fn     bool_t ARP_Open(ifHandle_t* pIfHandle, arpSend sendFp)
\brief  Interface function for the ARP module. It starts the ARP on an interface

\param [in]    pIfHandle      double pointer to media interface configuration structure

\retval        none 
***************************************************************************************************/
uint32_t ARP_Open(ifHandle_t* pIfHandle);
/*!*************************************************************************************************
\fn     bool_t ARP_Close(ifHandle_t* pIfHandle)
\brief  Interface function for the ARP module. It stops the ARP on an interface

\param  [in]  pIfHandle     double pointer to media interface configuration structure

\retval       none
***************************************************************************************************/
bool_t ARP_Close(ifHandle_t* pIfHandle);
/*!*************************************************************************************************
\fn     void ARP_ManageCache(uint32_t timerInterval)
\brief  Manages the ARP cache entries.

\param  [in]    timerInterval      The time interval that passed since the last call of the func.
***************************************************************************************************/
void ARP_ManageCache(uint32_t timerInterval);


#ifdef __cplusplus
}
#endif

#endif
/*================================================================================================*/
#endif  /*_ARP_H_ */
