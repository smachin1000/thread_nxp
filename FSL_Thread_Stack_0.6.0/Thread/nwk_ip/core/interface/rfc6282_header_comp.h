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

#ifndef _RFC_6282_HEADER_COMP_H
#define _RFC_6282_HEADER_COMP_H
/*!=================================================================================================
\file       rfc6282_header_comp.h
\brief      This is a header file for the RFC 6282 module. It contains the implementation for 
            6LowPan Header Compression Protocol (6LowPanHC).
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "sixlowpan.h"

#if 0
#include <stdint.h>
#include "adp_util.h"
#endif
/*==================================================================================================
Public macros
==================================================================================================*/


/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef enum
{
    gHeadCompSucces_c = 0U,
    gHeadCompFailed_c,    
    gHeadCompSizeExceeded_c,
    gHeadCompContinueComp_c,
    gHeadCompContinueDecomp_c,
    gHeadCompBadContext_c,
}headCompResultType_t;

typedef struct decompInfo_tag
{
    uuint64_t       srcIID;
    uuint64_t       dstIID;
    nwkBuffer_t *   pDecompBuff;
    uint8_t *       pDecompPos;
    uint8_t         nextHeader;
} decompInfo_t;

/* address compression options structure */
typedef struct addrCompResult_tag
{
  uint8_t   addrMode;
  uint8_t   addrCmpr;
  uint8_t   cid;
}addrCompOpt_t;

#if 0
typedef struct RFC6282_buffInfo_tag
{
  uint8_t*  pBuffer;
  uint32_t bufferSize;
}RFC6282_buffInfo_t;

typedef struct contextInfo_tag
{
  uint32_t  validLifetime;
  uint8_t   contextPrefix[16];  
  uint8_t   contextLength;
  uint8_t   cid;
  uint16_t  usedForContextComp;
}contextInfo_t;
#endif

/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

void RFC6282_CompressPacket(slwpPktInfo_t * pSlwpPktInfo);
bool_t RFC6282_ProcessIPHC(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos);
bool_t RFC6282_ProcessSingleFragIPHC(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos);
uint32_t RFC6282_GetDecompressedSize(macAbsMcpsDataInd_t * pMcpsDataInd);
headCompResultType_t RFC6282_DecompressPacket(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos, decompInfo_t * pDecompInfo);
headCompResultType_t RFC6282_DecompressIpHeader(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos, decompInfo_t * pDecompInfo);
headCompResultType_t RFC6282_DecompressExtentionHeader(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos, decompInfo_t * pDecompInfo);
headCompResultType_t RFC6282_DecompressUdpHeader(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos, decompInfo_t * pDecompInfo);
headCompResultType_t RFC6282_DecompressUnicastAddress(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos, uuint64_t * pIID, uint8_t * pIpAddr, addrCompOpt_t * pAddrCompOpt);
headCompResultType_t RFC6282_DecompressMulticastAddress(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos, uuint64_t * pIID, uint8_t * pIpAddr, addrCompOpt_t * pAddrCompOpt);

/*!*************************************************************************************************
\fn RFC6282_ComputeLenghtAndUdpChecksum(macAbsMcpsDataInd_t * pMcpsDataInd)
\brief  Recalculates IP lenght and UDP checksum.

\param [in]   pMcpsDataInd      pointer to the mcps data indication

\retval       none
***************************************************************************************************/
void RFC6282_ComputeLenghtAndUdpChecksum(macAbsMcpsDataInd_t * pMcpsDataInd);


/*!*************************************************************************************************
\fn bool RFC6282_IsIPHC(uint8_t* pInData) 
\brief  Checks if data is a IPHC

\param [in]   pInData       pointer to input data
       
\retval       bool          TRUE or FALSE (operation status)
***************************************************************************************************/
bool_t RFC6282_IsIPHC(uint8_t* pInData);

#if 0
/*!*************************************************************************************************
\fn uint32_t RFC6282_CompressPacket(RFC6282_buffInfo_t* pInData, macAddrInfo_t* srcInfo,
           macAddrInfo_t* dstInfo,  uint8_t** ppOutUncompressedOffset, RFC6282_buffInfo_t* pOutData)
\brief  Compress IpV6 packet.

\param [in]   pInData                   input data pointer + size
\param [in]   srcInfo                   source info
\param [in]   dstInfo                   destination info
\param [out]  ppOutUncompressedOffset   offset in the uncompressed packet where the compression 
                                        finished
\param [out]  pOutData                  output data pointer + size

\retval       uint32_t                  the compressed size
***************************************************************************************************/
uint32_t RFC6282_CompressPacket( RFC6282_buffInfo_t* pInData, macAddrInfo_t* srcInfo,
            macAddrInfo_t* dstInfo, uint8_t ** ppOutUncompressedOffset,  RFC6282_buffInfo_t* pOutData);
#endif

#if 0
/*!*************************************************************************************************
\fn uint32_t RFC6282_DecompressPacket(RFC6282_buffInfo_t* pInData, macAddrInfo_t* srcInfo,
           macAddrInfo_t* dstInfo, RFC6282_buffInfo_t* pOutData,uint8_t** ppStartOfUncompData )
\brief  Decompress IpV6 packet.

\param [in]   pInData                   input data pointer + size
\param [in]   srcInfo                   source info
\param [in]   dstInfo                   destination info
\param [out]  pOutData                  output data pointer + size
\param [out]  ppStartOfUncompData       offset in the uncompressed packet where the compression 
                                        finished

\retval       headCompResultType_t      gHeadCompSucces_c - success
                                        gHeadCompBadContext_c - context not found
***************************************************************************************************/
headCompResultType_t RFC6282_DecompressPacket(RFC6282_buffInfo_t* pInData, 
                  macAddrInfo_t* srcInfo, macAddrInfo_t* dstInfo, RFC6282_buffInfo_t* pOutData,
                  uint8_t** ppStartOfUncompData);
#endif

#if 0
/*!*************************************************************************************************
\fn    bool RFC6282_SetContextTableEntry(contextInfo_t* contextEntry) 
\brief  Adds a context table entry.

\param [in]   contextEntry      context table entry struct
       
\retval       bool              TRUE or FALSE (operation status)
***************************************************************************************************/
bool RFC6282_SetContextTableEntry(contextInfo_t* contextEntry);
/*!*************************************************************************************************
\fn    bool RFC6282_GetContextTableEntry(uint32_t index, contextInfo_t* contextEntry) 
\brief  Gets a context table entry.

\param [in]   index             context table index
\param [in]   contextEntry      context table entry struct
       
\retval       bool              TRUE or FALSE (operation status)
***************************************************************************************************/
bool RFC6282_GetContextTableEntry(uint32_t index, contextInfo_t* contextEntry);
/*!*************************************************************************************************
\fn    void RFC6282_ResetContextTable() 
\brief  Resets the context table.
     
\retval       none
***************************************************************************************************/
void RFC6282_ResetContextTable(void);

#endif

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*_RFC_6282_HEADER_COMP_H */
