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

#ifndef _MLE_EXTENDED_H
#define _MLE_EXTENDED_H
/*!=================================================================================================
\file       mle_extended.h
\brief      This is a header file for the Mesh Link Establishment module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"

/* Network Includes */
#include "network_utils.h"

/*==================================================================================================
Public macros
==================================================================================================*/

#define TLV_ROUTE_ROUTER_OUT_MASK       0xC0

#define TLV_ROUTE_ROUTER_OUT_SHIFT      6

#define TLV_ROUTE_ROUTER_IN_MASK        0x30

#define TLV_ROUTE_ROUTER_IN_SHIFT       4

#define TLV_ROUTE_ROUTER_ROUTE_MASK     0xF

/*==================================================================================================
Public type definitions
==================================================================================================*/

/*! MLE Extended Command Types Enumeration */
typedef enum mleCmdExtType_tag
{
    gMleCmdDataReq_c            = 7U,
    gMleCmdDataRes_c            = 8U,
    gMleCmdParentReq_c          = 9U,
    gMleCmdParentRes_c          = 10U,
    gMleCmdChildIdReq_c         = 11U,
    gMleCmdChildIdRes_c         = 12U
} mleCmdExtType_e;

/*! MLE Extended TLV Types Enumeration */
typedef enum melTlvExtType_tag
{
    gMleTlvRoute_c              = 9U,
    gMleTlvAddress16_c          = 10U,              /*!< Contains a 16-bit MAC address */
    gMleTlvLeaderData_c         = 11U,
    gMleTlvNetworkData_c        = 12U,
    gMleTlvTlvRequest_c         = 13U,
    gMleTlvScanMask_c           = 14U,
    gMleTlvConnectivity_c       = 15U,
    gMleTlvRssi_c               = 16U,
} mleTlvExtType_e;

typedef uint8_t mleTlvExtType_t;

/******************/
/* Address 16 TLV */
/******************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |           Address             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

*******************/

/*! Address 16 TLV - Tx Interface structure */
typedef struct mleTlvAddress16_tag
{
    mleTlvExtType_t tlvType;
    uint16_t address;
} mleTlvAddress16_t;

/*! Address 16 TLV - Over the Air mapping structure */
typedef struct mleOtaTlvAddress16_tag
{   
    uint8_t type;
    uint8_t length;
    uint8_t shortAddr[2];
} mleOtaTlvAddress16_t;

/*******************/
/* Leader Data TLV */
/*******************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|    TLV Type   |     Length    |          Instance ID  
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
                                |   Weighting   | Data Version  |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|Stable Data Ver|   Leader ID   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

********************/

/*! Leader Data TLV - Tx Interface structure */
typedef struct mleTlvLeaderData_tag
{
    mleTlvExtType_t tlvType;
    uint32_t instanceId;                    /*!< Network Segment Identifier */
    uint8_t  weighting;                     /*!< Weighting value for the network fragment */
    uint8_t  dataVersion;                   /*!< Version of the Network Data */
    uint8_t  stableDataVersion;             /*!< Stable Version of the Network Data */
    uint8_t  leaderId;                      /*!< Network Leader Router ID */
} mleTlvLeaderData_t;

/*! Leader Data TLV - Over the Air mapping structure */
typedef struct mleOtaTlvLeaderData_tag
{
    uint8_t type;
    uint8_t length;
    uint8_t instanceId[4];                  /*!< Network Segment Identifier */
    uint8_t weighting;                      /*!< Weighting value for the network fragment */
    uint8_t dataVersion;                    /*!< Version of the Network Data */
    uint8_t stableDataVersion;              /*!< Stable Version of the Network Data */
    uint8_t leaderId;                       /*!< Network Leader Router ID */
} mleOtaTlvLeaderData_t;



/****************/
/* NWK Data TLV */
/****************

 0                   1                   2        
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3  
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |    NWK DATA
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

****************/

/*! NWK Data TLV - Over the Air mapping structure */
typedef struct mleOtaTlvNwkData_tag
{
    uint8_t type;
    uint8_t length;
    uint8_t data[];
} mleOtaTlvNwkData_t;

/*******************/
/* Tlv Request TLV */
/*******************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |    TLV ID 0   |    TLV ID 1   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
...                                                          
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

********************/

/*! Tlv Request TLV - Tx Interface structure */
typedef struct mleTlvTlvRequest_tag
{
    mleTlvExtType_t tlvType;
    uint32_t nbTlvs;
    uint8_t  aTlvReqIds[];
} mleTlvTlvRequest_t;

/*! Tlv Request TLV - Over the Air mapping structure */
typedef struct mleOtaTlvTlvRequest_tag
{
    uint8_t type;
    uint8_t length;
    uint8_t aTlvReqIds[];
} mleOtaTlvTlvRequest_t;

/*****************/
/* Scan Mask TLV */
/*****************

 0                   1                   2        
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3  
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |     Length    |R|E|  Reserved |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

******************/

/*! Scan Mask Flags bit masks */
typedef enum mleTlvScanMaskFlags_tag
{
    gMleTlvScanMaskFlagRouter_c = 0x80U,            /*!< Active Routers flag */
    gMleTlvScanMaskFlagEligibleRouter_c = 0x40U,    /*!< Router Capable End Devices flag */
} mleTlvScanMaskFlags_e;

/*! Scan Mask TLV - Tx Interface structure */
typedef struct mleTlvScanMask_tag
{
    mleTlvExtType_t tlvType;
    uint8_t scanMask;
} mleTlvScanMask_t;

/*! Scan Mask TLV - Over the Air mapping structure */
typedef struct mleOtaTlvScanMask_tag
{
    uint8_t type;
    uint8_t length;
    uint8_t scanMask;
} mleOtaTlvScanMask_t;

/********************/
/* Connectivity TLV */
/********************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|Max Child Count|  Child Count  |Link Quality 3 |Link Quality 2 |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|LinkQuality 1  |
+-+-+-+-+-+-+-+-+

*********************/

/*! Connectivity TLV - Tx Interface structure */
typedef struct mleTlvConectivity_tag
{
    mleTlvExtType_t tlvType;
    uint8_t maxChildCount;
    uint8_t childCount;
    uint8_t cLinkQuality3;      /* Number of connected children with link quality 3 */
    uint8_t cLinkQuality2;
    uint8_t cLinkQuality1;
} mleTlvConnectivity_t;

/*! Connectivity TLV - Over the Air mapping structure */
typedef struct mleOtaTlvConectivity_tag
{
    uint8_t type;
    uint8_t length;
    uint8_t maxChildCount;
    uint8_t childCount;
    uint8_t cLinkQuality3;      /* Number of connected children with link quality 3 */
    uint8_t cLinkQuality2;
    uint8_t cLinkQuality1;    
} mleOtaTlvConnectivity_t;

/************/
/* RSSI TLV */
/************

 0                   1                   2        
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3  
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |     RSSI      |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

*************/

/*! RSSI TLV - Tx Interface structure */
typedef struct mleTlvRssi_tag
{
    mleTlvExtType_t tlvType;
    uint8_t rssi;
} mleTlvRssi_t;

/*! RSSI TLV - Over the Air mapping structure */
typedef struct mleOtaTlvRssi_tag
{
    uint8_t type;
    uint8_t length;
    uint8_t rssi;
} mleOtaTlvRssi_t;

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
\fn     void MLE_TlvAddress16Add(void *pParams, list_t *pTlvList)
\brief  Function used to add the Short Address TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvAddress16Add(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvLeaderDataAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Leader Data TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvLeaderDataAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvTlvRequestAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the TLV Request TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvTlvRequestAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvScanMaskAdd(void *pParams, list_t *pTlvList)
\brief  Function used to.

\param  [in]    pParams         Pointer to the TLV in the received MLE packet.
\param  [in]    pTlvList        Pointer to the TLV list.

\return         uint32_t        Size of the TLV.
***************************************************************************************************/
void MLE_TlvScanMaskAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvConnectivityAdd(void *pParams, list_t *pTlvList)
\brief  Function used to.

\param  [in]    pParams         Pointer to the TLV in the received MLE packet.
\param  [in]    pTlvList        Pointer to the TLV list.

\return         uint32_t        Size of the TLV.
***************************************************************************************************/
void MLE_TlvConnectivityAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvRssiAdd(void *pParams, list_t *pTlvList)
\brief  Function used to.

\param  [in]    pParams         Pointer to the TLV in the received MLE packet.
\param  [in]    pTlvList        Pointer to the TLV list.

\return         uint32_t        Size of the TLV.
***************************************************************************************************/
void MLE_TlvRssiAdd(void *pParams, list_t *pTlvList);

#if 0

/*!*************************************************************************************************
\fn     void MLE_TlvRouteAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Route TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvRouteAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvDefaultRouteAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Default Route TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvDefaultRouteAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvServerDataAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Server Data TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvServerDataAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvPrefixAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Prefix TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvPrefixAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvContextIdAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Context TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvContextIdAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvDhcpServerAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the DHCP Server TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvDhcpServerAdd(void *pParams, list_t *pTlvList);

#endif

#ifdef __cplusplus
}
#endif

/*================================================================================================*/
#endif  /* _MLE_EXTENDED_H */
