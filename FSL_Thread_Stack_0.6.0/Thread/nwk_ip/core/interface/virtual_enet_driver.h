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

#ifndef _VIRTUAL_ENET_DRIVER_H
#define _VIRTUAL_ENET_DRIVER_H
/*!=================================================================================================
\file       virtual_enet_driver.h
\brief      This is a header file for the Virtual enet media interface. 
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "network_utils.h"
#include "i2c_abstraction.h"
/*==================================================================================================
Public macros
==================================================================================================*/


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

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_get_address(uint8_t* address)
\brief  Retrieves the Ethernet address of a device.
        
\param [out]  address      mac address
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_get_address( uint8_t* address);

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_initialize(uint8_t* address)
\brief  Initializes the chip.
        
\param [in]  address       the local Ethernet address
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_initialize(uint8_t* address);   
/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_open(uint16_t protocol, void (*service)(uint8_t *, uint32_t))
\brief  Registers a protocol type on an Ethernet channel.

\param [in]  protocol      the protocol to open
\param [in]  service       the callback function
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_open(uint16_t protocol, void (*service)(uint8_t *, uint32_t));
/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_close(uint16_t protocol)
\brief  Unregisters a protocol type on an Ethernet channel.

\param [in]  protocol      the protocol to close
      
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_close();
/*!*************************************************************************************************
\fn void VIRTUAL_ENET_receive(uint8_t* inData,  uint32_t inDataLen)
\brief  Enet receive callback function.

\param [in]  inData       received data
\param [in]  inDataLen    received data lenght
      
\retval      none
***************************************************************************************************/
void VIRTUAL_ENET_receive(uint8_t* inData, uint32_t inDataLen);

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_send(ipPktInfo_t* packet,uint16_t protocol, uint8_t* dest, uint32_t  flags)
\brief  Sends a packet.

\param [in]  packet        the packet to send
\param [in]  protocol      the protocol to send
\param [in]  dest          the destination Ethernet address
\param [in]  flags         optional flags, zero = default
      
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_send(ipPktInfo_t* packet, uint16_t protocol,uint8_t*  dest, 
                                    uint32_t  flags);
/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_get_MTU()
\brief  Get the maximum transmission unit.
      
\retval       uint32_t     ENET MTU
***************************************************************************************************/
uint32_t VIRTUAL_ENET_get_MTU();
/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_join(uint8_t* address,uint16_t protocol)
\brief  Joins a multicast group on an Ethernet channel.

\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_join(uint8_t* address, uint16_t protocol);
/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_leave(uint8_t* address, uint16_t protocol)
\brief  Leaves a multicast group on an Ethernet channel.

\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_leave(uint8_t* address, uint16_t protocol);
/*!*************************************************************************************************
\fn void VIRTUAL_ENET_reset()
\brief  Resets the I2C to ETH adapter

\retval      none              
***************************************************************************************************/
void VIRTUAL_ENET_reset(void);
/*!*************************************************************************************************
\fn void VIRTUAL_ENET_process(i2cHdr_t *pHeader,uint8_t *pBuffer)
\brief  Handles the return value from a virtual enet function call.

\param [in]  pHeader       the packet header
\param [in]  pBuffer       data pointer
      
\retval      none
***************************************************************************************************/
void VIRTUAL_ENET_process(i2cHdr_t *pHeader, uint8_t *pBuffer);
#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*_VIRTUAL_ENET_DRIVER_H */
