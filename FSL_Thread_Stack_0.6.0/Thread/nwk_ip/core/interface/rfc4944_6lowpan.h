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

#ifndef _RFC4944_6LOWPAN_H
#define _RFC4944_6LOWPAN_H
/*!=================================================================================================
\file       rfc4944_6lowpan.h
\brief      This is a header interface file for the RFC4944 (Transmission of IPv6 Packets over 
            IEEE 802.15.4 Networks, September 2007) module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "EmbeddedTypes.h"
#include "mac_abs_types.h"
#include "network_utils.h"
#include "sixlowpan_interface.h"
#include "sixlowpan_tbl.h"

/*==================================================================================================
Public macros
==================================================================================================*/

#define RFC4944_DISPATCH_SIZE           1
#define RFC4944_MESH_HEADER_SIZE        RFC4944_DISPATCH_SIZE + 4
#define RFC4944_MAX_MESH_HEADER_SIZE    RFC4944_DISPATCH_SIZE + 16
#define RFC4944_BROACAST_HEADER_SIZE    RFC4944_DISPATCH_SIZE + 1
#define RFC4944_FRAG1_HEADER_SIZE       RFC4944_DISPATCH_SIZE + 3
#define RFC4944_FRAGN_HEADER_SIZE       RFC4944_DISPATCH_SIZE + 4
#define RFC4944_IPv6_DISPATCH_SIZE      RFC4944_DISPATCH_SIZE
#define RFC4944_HC1_DISPATCH_SIZE       RFC4944_DISPATCH_SIZE
#define RFC4944_ESC_DISPATCH_SIZE       RFC4944_DISPATCH_SIZE

#define RFC4944_DISPATCH_NALP_MASK              (0xFFU >> 2)
#define RFC4944_DISPATCH_IPV6_MASK              0xFFU
#define RFC4944_DISPATCH_LOWPAN_HC1_MASK        0xFFU
#define RFC4944_DISPATCH_LOWPAN_BC0_MASK        0xFFU
#define RFC4944_DISPATCH_ESC_MASK               0xFFU
#define RFC4944_DISPATCH_MESH_MASK              (0x03U << 6)
#define RFC4944_DISPATCH_FRAG1_MASK             (0x1FU << 3)
#define RFC4944_DISPATCH_FRAGN_MASK             (0x1FU << 3)

#define RFC4944_DISPATCH_NALP_TEMPLATE          0x00U
#define RFC4944_DISPATCH_IPV6_TEMPLATE          0x41U
#define RFC4944_DISPATCH_LOWPAN_HC1_TEMPLATE    0x42U
#define RFC4944_DISPATCH_LOWPAN_BC0_TEMPLATE    0x50U
#if (SLWPCFG_RFC6282_COMPRESSION_ENABLED)
#define RFC4944_DISPATCH_ESC_TEMPLATE           0x40U
#else       
#define RFC4944_DISPATCH_ESC_TEMPLATE           0x7FU
#endif
#define RFC4944_DISPATCH_MESH_TEMPLATE          0x80U
#define RFC4944_DISPATCH_FRAG1_TEMPLATE         0xC0U
#define RFC4944_DISPATCH_FRAG_HEADER_MASK       0xF8U
#define RFC4944_DISPATCH_FRAGN_TEMPLATE         0xE0U

#define RFC4944_FRAG_OPT_OFFSET_DISP           0x01
#define RFC4944_FRAG_OPT_IPHC                   0x02

#define RFC4944_MESH_HEADER_V_SHIFT             5U
#define RFC4944_MESH_HEADER_F_SHIFT             4U
#define RFC4944_MESH_HEADER_V_MASK              1U
#define RFC4944_MESH_HEADER_F_MASK              1U
#define RFC4944_MESH_HEADER_HOPS_LEFT_MASK      0x0FU

#define BROADCAST_HEADER_SEQ_NB_OFFSET          1U

#define IPV6_UNCOMPRESSED_HEADER_SIZE           40U

///TODO: Move to appropriate location *CCotiga
#define IPV6_VERSION                            6U
#define IPV6_ADDRESS_SIZE_IN_BYTES              16U
#define IPV6_DEST_ADDR_SHORT_OFFSET             14U
#define IPV6_DEST_ADDR_PANID_OFFSET             8U

#define IPV6_VERSION_OFFSET                     0U
#define IPV6_VERSION_MASK                       0x0FU
#define IPV6_VERSION_SHIFT                      4U
#define IPV6_TRAFFIC_CLASS_OFFSET               0U
#define IPV6_TRAFFIC_CLASS_MASK                 0xFF
#define IPV6_TRAFFIC_CLASS_SHIFT                20U
#define IPV6_FLOW_LABEL_OFFSET                  0U
#define IPV6_FLOW_LABEL_MASK                    0xFFFFFU
#define IPV6_FLOW_LABEL_SHIFT                   0U
#define IPV6_PLOAD_LEN_OFFSET                   4U
#define IPV6_PLOAD_LEN_MASK                     0xFFU
#define IPV6_PLOAD_LEN_SHIFT                    0U
#define IPV6_NEXT_HEADER_OFFSET                 4U              
#define IPV6_NEXT_HEADER_MASK                   0xFFU
#define IPV6_NEXT_HEADER_SHIFT                  8U
#define IPV6_HOP_LIMIT_OFFSET                   4U
#define IPV6_HOP_LIMIT_MASK                     0xFFU
#define IPV6_HOP_LIMIT_SHIFT                    0
#define IPV6_SRC_ADDR_OFFSET                    8U
#define IPV6_DST_ADDR_OFFSET                    24U

#define ADP_MULTICAST_ALL_NODES_ADDRESS         0x8001U
#define ADP_MULTICAST_ALL_ROUTERS_ADDRESS       0x8002U

#define ADP_MESH_MULTICAST                      0x8000U
#define ADP_MESH_MULTICAST_MASK                 0x1FFFU

#define ADP_LINK_LAYER_BROADCAST_ADDRESS        0xFFFFU

#define ADP_MAX_UNICAST_ADDRESS                 0x7FFFU

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*! Enumeration of RFC4944 dispatch types */
typedef enum rfc4944DispatchType_tag
{
    gRfc4944DispatchMin_c               = 0U,       /*!< Reserved 6LoWPAN dispatch. Not used. */
    gRfc4944DispatchMesh_c              = 1U,       /*!< Mesh Header */
    gRfc4944DispatchLowpanBC0_c         = 2U,       /*!< LOWPAN_BC0 broadcast */        
    gRfc4944DispatchFrag1_c             = 4U,       /*!< Fragmentation Header (first) */
    gRfc4944DispatchFragN_c             = 8U,       /*!< Fragmentation Header (subsequent) */
    gRfc4944DispatchLowpanHC1_c         = 16U,      /*!< LOWPAN_HC1 compressed IPv6 */
    gRfc4944DispatchIPv6_c              = 32U,      /*!< Uncompressed IPv6 Addresses */
    gRfc4944DispatchEsc_c               = 64U,      /*!< Additional Dispatch byte follows */   
    gRfc4944DispatchNALP_c              = 128U,     /*!< Not a LoWPAN frame */
    gRfc4944DispatchReserved_c          = 256U,     /*!< Reserved 6LoWPAN dispatch. Not used. */    
} rfc4944DispType_t;

/*! Enumeration of G3 6LoWPAN command IDs */
typedef enum escCommandId_tag
{
    gEscCmdIdMeshRoutingMsg_c    = 0x01U,              /*!< Mesh routing protocol message */
    gEscCmdIdLbpMsg_c            = 0x02U,              /*!< 6LoWPAN bootstraping procedure message */
    gEscCmdIdCfaMsg_c            = 0x03U,              /*!< Contention Free Access message */
} escCommandId_t;

typedef struct fragHeader_tag
{
    uint8_t datagramSize[2];
    uint8_t datagramTag[2];
    uint8_t datagramOffset;
} fragHeader_t;

typedef struct iidInfo_tag
{
    uuint64_t       srcIID;
    uuint64_t       dstIID;
} iidInfo_t;

#if 0
typedef struct broadcastHeader_tag
{
    uint8_t  broadcastDispatch;    
    uint8_t  sequenceNumber;
} broadcastHeader_t;

typedef enum
{
    gMeshStatusInputError_c,
    gMeshStatusForward_c,
    gMeshStatusMulticast_c,                         /* Unicast trasmission */
    gMeshStatusDrop_c,                              /* Multicast transmission */
    gMeshStatusOwn_c                                /* Broadcast transmission */   
} meshStatus_t;

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

adpStatus_t RFC4944_GetLinkLayerInfo(slwpPktInfo_t * pSlwpPktInfo, ipAddr_t * pIpDstAddr);

bool_t RFC4944_CreateMeshHeader(slwpPktInfo_t * pSlwpPktInfo);
void RFC4944_CreateBCastHeader(slwpPktInfo_t * pSlwpPktInfo);
void RFC4944_CompressPacket(slwpPktInfo_t * pSlwpPktInfo);
uint32_t RFC4944_GetTotalPcktSize(slwpPktInfo_t * pSlwpPktInfo);

bool_t RFC4944_FragInfoAddDGramOffset(fragInfo_t * pFragInfo, dgramOffset_t offset);
void RFC4944_FragInfoInsertDGramOffset(fragInfo_t * pFragInfo, dgramOffset_t offset, uint32_t pos);

bool_t RFC4944_ProcessIPv6Packet(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos);
bool_t RFC4944_ProcessEscDispatch (macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos);

#if (SLWPCFG_MESH_ENABLED)
bool_t RFC4944_ProcessMeshHeader(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos);
bool_t RFC4944_ProcessBCastHeader(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos);
#endif

bool_t RFC4944_ProcessFragPacket(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos, 
                                               rfc4944DispType_t dispType);
bool_t RFC4944_ProcessHC1(macAbsMcpsDataInd_t * pMcpsDataInd, uint8_t ** ppCrtPos);

bool_t RFC4944_IsMeshNeeded(void * param);


/*!*************************************************************************************************
\fn    bool_t RFC4944_GetDispatchType(uint8_t * pData, rfc4944DispType_t * crtDispType,
                                      rfc4944DispType_t * prvDispType);
\brief Public interface function for the RFC4944 module. This function analyzes the input and 
       determines the type of 6LoWPAN dispatch. If present, the dispatch is removed and the pDataInOut
       pointer is set to point to the following header. If not present or if the dispatch order is 
       wrong, the function returns false.

\param [in,out] pData                   Pointer to input frame which will be modified after the 
                                        removal of the dispatch.
\param [in,out] crtDispType             The dispatch type is written to the address.
\param [out]    prvDispType             The last processed dispatch type is written to the address.

\return         boolean                 False in case of unknown dispatch or dispatch was received
                                        in wrong order.
***************************************************************************************************/
bool_t RFC4944_GetDispatchType(uint8_t * pData, rfc4944DispType_t * crtDispType,
                                           rfc4944DispType_t * prvDispType);

/*!*************************************************************************************************
\fn     void RFC4944_GetCompressionIID(slwpPktInfo_t * pSlwpPktInfo, iidInfo_t * pIID)
\brief  Interface function for the 6LoWPAN module. It returns the Link-Layer address from the IID.

\param  [in]     pSlwpPktInfo    Pointer to 6LoWPAN packet structure
\param  [in,out] pIID            Pointer to the IID
***************************************************************************************************/
void RFC4944_GetCompressionIID(slwpPktInfo_t * pSlwpPktInfo, iidInfo_t * pIID);

#if 0
/*!*************************************************************************************************
\fn    bool RFC4944_AddEscDispatch(uint8_t * pDataIn)
\brief Public interface function for the RFC4944 module. This function adds an ESC dispatch in front
       of the 6LoWPAN packet.

\param [in]   pDataIn                     Pointer to the location where to copy the ESC dispatch.

\return       bool                        Returns false if the input pointer is NULL.
***************************************************************************************************/
bool RFC4944_AddEscDispatch(uint8_t * pDataIn);

/*!*************************************************************************************************
\fn    bool RFC4944_AddBcastHeader(uint8_t * pDataIn)
\brief Public interface function for the RFC4944 module. This function adds an broadcast header to a
       6LoWPAN packet.

\param [in]   pDataIn           Pointer to the location where to copy the broadcast header.

\return       bool              Returns false if the adding of the header failed.
***************************************************************************************************/
bool RFC4944_AddBcastHeader(uint8_t * pDataIn);

/*!*************************************************************************************************
\fn    bool RFC4944_CreateMeshHeader(uint8_t * pDataIn, uint8_t adpMaxHops, uint16_t originatorAddress,
                                  uint16_t destinationAddress)
\brief Public interface function for the RFC4944 module. This function adds a MESH header to a
       6LoWPAN packet.

\param [in]   pDataIn           Pointer to the location where to copy the MESH header.

\return       bool              Returns false if the adding of the header failed.
***************************************************************************************************/
bool RFC4944_CreateMeshHeader(    uint8_t * pDataIn,         uint8_t adpMaxHops, macAddrInfo_t * pOriginatorAddress,
                                       macAddrInfo_t * pFinalAddress);

/*!*************************************************************************************************
\fn     void RFC4944_ExtractBroadcastHeader(uint8_t ** ppCurrentIndex,
                                            uint32_t * pRemainingLength,
                                            uint8_t *  pBCastSeqNumber)
\brief  Public interface function for the RFC4944 module. This function processes a MESH header in a
        6LoWPAN packet.

\param  [in|out]    ppCurrentIndex      Pointer to the start of the BROADCAST header.
\param  [in|out]    pRemainingLength    Pointer to where to write the remaining packet size.
\param  [out]       pBCastSeqNumber     Pointer to the location where to copy the BCAST header info.
***************************************************************************************************/
void RFC4944_ExtractBroadcastHeader(uint8_t ** ppCurrentIndex, uint32_t * pRemainingLength, uint8_t * pBCastSeqNumber);

/*!*************************************************************************************************
\fn    bool RFC4944_CreateNextFragment(uint8_t * pDataIn)
\brief Public interface function for the RFC4944 module. This function creates the next fragment of
       a packet which is bigger than the maximum transmission length supported by the lower layer.
       The function uses the input structure to store its information for the next time is is called.

\param [in]   pTransmissionInfo           Pointer to the transmission information structure.

\return       bool                        Returns false if no more fragments are available.
***************************************************************************************************/
bool RFC4944_CreateNextFragment(slwpPktInfo_t * pTransmissionInfo);

/*!*************************************************************************************************
\fn     void RFC4944_ExtractFrag1Header(uint8_t ** ppCurrentIndex,
                                        uint32_t * pRemainingLength,
                                        uint32_t * pFullPacketSize,
                                        uint16_t * pDatagramTag)
\brief  Public interface function for the RFC4944 module. This function extracts the first fragment
        information.

\param  [in|out]    ppCurrentIndex      Pointer to the start of the BROADCAST header.
\param  [in|out]    pRemainingLength    Pointer to where to write the remaining packet size.
\param  [out]       pFullPacketSize     Pointer to where to write the full packet size.
\param  [out]       pDatagramTag        Pointer to where to write the fragment datagram tag.
***************************************************************************************************/
void RFC4944_ExtractFrag1Header(uint8_t ** ppCurrentIndex, uint32_t * pRemainingLength,
                                uint32_t * pFullPacketSize, uint16_t * pDatagramTag);

/*!*************************************************************************************************
\fn     void RFC4944_ExtractFragNHeader(uint8_t ** ppCurrentIndex,
                                        uint32_t * pRemainingLength,
                                        uint32_t * pFullPacketSize,
                                        uint16_t * pDatagramTag,
                                        uint8_t  * pDatagramOffset)
\brief  Public interface function for the RFC4944 module. This function extracts the subsequent
        fragment information.

\param  [in|out]    ppCurrentIndex      Pointer to the start of the BROADCAST header.
\param  [in|out]    pRemainingLength    Pointer to where to write the remaining packet size.
\param  [out]       pFullPacketSize     Pointer to where to write the full packet size.
\param  [out]       pDatagramTag        Pointer to where to write the fragment datagram tag.
\param  [out]       pDatagramOffset     Pointer to where to write the fragment datagram offset.
***************************************************************************************************/
void RFC4944_ExtractFragNHeader(uint8_t ** ppCurrentIndex, uint32_t * pRemainingLength, 
                                uint32_t * pFullPacketSize, uint16_t * pDatagramTag, uint8_t * pDatagramOffset);

#if (ADPCFG_TEST_FEATURES_ENABLED)

/*!*************************************************************************************************
\fn     void RFC4944_SetBroadcastSeqNum(uint8_t seqNumber)
\brief  Sets a new value for the broadcast sequence number

\author B36295
\date   19-Apr-2013

\param  [in]   seqNumber        new broadcast sequence number value

\return        none
***************************************************************************************************/
void RFC4944_SetBroadcastSeqNum(uint8_t seqNumber);

/*!*************************************************************************************************
\fn     void RFC4944_SetFragDatagramTag(uint16_t datagramTag)
\brief  Sets a new value for the fragmentation datagram tag

\author B36295
\date   19-Apr-2013

\param  [in]   datagramTag        new fragmentation datagram tag value

\return        none
***************************************************************************************************/
void RFC4944_SetFragDatagramTag(uint16_t datagramTag);

#endif /* ADPCFG_TEST_FEATURES_ENABLED */
#endif

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /* _RFC4944_6LOWPAN_H */
