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

#ifndef _MLE_BASIC_H
#define _MLE_BASIC_H
/*!=================================================================================================
\file       mle_basic.h
\brief      This is a header file for the Mesh Link Establishment module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"

/* Network Includes */
#include "network_utils.h"

#include "mle.h"

/*==================================================================================================
Public macros
==================================================================================================*/



/*==================================================================================================
Public type definitions
==================================================================================================*/

/*! MLE Command Type enumeration */
typedef enum mleCmdType_tag
{
    gMleCmdLinkReq_c            = 0U,
    gMleCmdLinkAccept_c         = 1U,
    gMleCmdLinkAcceptAndReq_c   = 2U,
    gMleCmdLinkReject_c         = 3U,
    gMleCmdAdvertisement_c      = 4U,
    gMleCmdUpdate_c             = 5U,
    gMleCmdUpdateReq_c          = 6U,
} mleCmdType_e;

/*! MLE TLV Type enumeration */
typedef enum mleTlvType_tag
{
    gMleTlvSrcAddr_c            = 0U,
    gMleTlvMode_c               = 1U,           
    gMleTlvTimeout_c            = 2U,
    gMleTlvChallenge_c          = 3U,
    gMleTlvResponse_c           = 4U,
    gMleTlvLlayerFrameCtr_c     = 5U,
    gMleTlvLinkQuality_c        = 6U,
    gMleTlvNetworkParam_c       = 7U,
    gMleTlvMleFrameCtr_c        = 8U,
} mleTlvType_e;

/**********************/
/* Source Address TLV */
/**********************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |             EUI...
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

***********************/

/*! Source Address TLV - Tx Interface structure */
typedef struct mleTlvSrcAddr_tag
{
    mleTlvType_t tlvType;
    llAddr_t address;
} mleTlvSrcAddr_t;

/*! Source Address TLV - Over the Air mapping structure */
typedef struct mleOtaTlvSrcAddr_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t eui[];
} mleOtaTlvSrcAddr_t;

/************/
/* Mode TLV */
/************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |   Capab Info  |S|N|  Reserved |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

*************/

/*! Device Capability Information type */
typedef uint8_t devCapabilityInfo_t;

/*! Device Capability Information bit masks */
typedef enum devCapabilityInfo_tag
{
    gDevCapInfoAltPanCoord_c        = 0x01U,    /*!< Alternate PAN Coordinator */
    gDevCapInfoDevType_c            = 0x02U,    /*!< Device Type */
    gDevCapInfoRxOnIdle_c           = 0x08U,    /*!< Receiver On When Idle */
    gDevCapInfoSecCapability_c      = 0x40U,    /*!< Security Capability */
    gDevCapInfoAllocAddress_c       = 0x80U,    /*!< Allocate Address */
} devCapabilityInfo_e;

/*! Mode Flags type */
typedef uint8_t modeFlags_t;

/*! Mode Flags bit masks */
typedef enum modeFlags_tag
{
    gModeFlagN_c                    = 0x01U,    /*!< Full Nwk Data flag */
    gModeFlagD_c                    = 0x02U,    /*!< Device Type FFD(1) or RFD(0) flag */
    gModeFlagS_c                    = 0x04U,    /*!< Secure Data Requests flag */
    gModeFlagR_c                    = 0x08U,    /*!< Receiver On When Idle flag */
} modeFlags_e;

/*! Mode TLV - Tx Interface structure */
typedef struct mleTlvMode_tag
{
    mleTlvType_t tlvType;
    uint8_t modeFlags;                          /*!< Mode flags: S (Secure Data Requests)
                                                                 N (Network Data) */
} mleTlvMode_t;

/*! Mode TLV - Over the Air mapping structure */
typedef struct mleOtaTlvMode_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t modeFlags;                          /*!< Mode flags: S (Secure Data Requests)
                                                                 N (Network Data) */
} mleOtaTlvMode_t;

/***************/
/* Timeout TLV */
/***************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |             Timeout           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|           Timeout             |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

****************/

/*! Timeout TLV - Tx Interface structure */
typedef struct mleTlvTimeout_tag
{
    mleTlvType_t tlvType;
    uint32_t timeout;
} mleTlvTimeout_t;

/*! Timeout TLV - Over the Air mapping structure */
typedef struct mleOtaTlvTimeout_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t timeout[4];
} mleOtaTlvTimeout_t;

/*****************/
/* Challenge TLV */
/*****************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |           Challenge ...          
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

******************/

/*! Challenge TLV - Tx Interface structure*/
typedef struct mleTlvChallenge_tag
{
    mleTlvType_t tlvType;
    uint8_t  challenge[16];
    uint8_t  size;
} mleTlvChallenge_t;

/*! Challenge TLV - Over the Air mapping structure */
typedef struct mleOtaTlvChallenge_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t challenge[];
} mleOtaTlvChallenge_t;

/****************/
/* Response TLV */
/****************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |           Response ...          
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

*****************/

/*! Response TLV - Tx Interface structure */
typedef struct mleTlvResponse_tag
{
    mleTlvType_t tlvType;
    uint8_t response[16];
    uint8_t size;
} mleTlvResponse_t;

/*! Response TLV - Over the Air mapping structure */
typedef struct mleOtaTlvResponse_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t response[];
} mleOtaTlvResponse_t;

/*********************/
/* Frame Counter TLV */
/*********************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  TLV Type 5   |    Length 1   |          Frame Counter        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|        Frame Counter          |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

**********************/

/*! Link Layer Frame Counter TLV - Tx Interface structure */
typedef struct mleTlvLlFrameCtr_tag
{
    mleTlvType_t tlvType;
    uint32_t frameCounter;
} mleTlvLlFrameCtr_t;

/*! Link Layer Frame Counter TLV - Over the Air mapping structure */
typedef struct mleOtaTlvLlFrameCtr_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t frameCounter[4];
} mleOtaTlvLlFrameCtr_t;

/********************/
/* Link Quality TLV */
/********************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  TLV Type 6   |    Length 1   |C| Res | Size  | Neighbor Data...
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

*********************/

/*! Link Quality TLV - Tx Interface structure */
typedef struct mleLinkQuality_tag
{
    mleTlvType_t tlvType;
    uint16_t complete;                      /*!< Has a value of 1 if the message includes all neighboring
                                                 routers for which the source has link quality data.
                                                 Otherwise is set to 0 */
    uint16_t size;                          /*!< The size of the type of included LL address minus 1.
                                                 Valid range: 0 - 15 */
    list_t   neighborDataList;              /*!< A list of neighbor records */
} mleLinkQuality_t;

/*! Link Quality TLV - Over the Air mapping structure */
typedef struct mleOtaLinkQuality_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t completeAndSize;
    uint8_t neighborData[];
} mleOtaLinkQuality_t;

/*************************/
/* Network Parameter TLV */
/*************************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  TLV Type 7   |    Length     | Parameter ID  |     Delay
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     Delay                     |    Value...
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

**************************/

/*! Network Parameter TLV Enumeration */
typedef enum nwkParamType_tag
{
    gNwkParamChannel_c,
    gNwkParamPanId_c,
    gNwkParamPermitJoining_c,
    gNwkParamBeaconPayload_c
} nwkParamType_e;

/*! Network Parameter TLV - Tx Interface structure */
typedef struct mleTlvNwkParam_tag
{
    mleTlvType_t        tlvType;
    uint32_t        delay;
    nwkParamType_t  paramId;
    uint16_t        size;
    uint8_t         aParam[];
} mleTlvNwkParam_t;

/*! Network Parameter TLV - Over the Air mapping structure */
typedef struct mleOtaTlvNwkParam_tag
{
    uint8_t         tlvType;
    uint8_t         length;
    uint8_t         paramId;
    uint8_t         delay[4];
    uint8_t         value[];
} mleOtaTlvNwkParam_t;

/*************************/
/* MLE Frame Counter TLV */
/*************************

 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|   TLV Type    |    Length     |          Frame Counter        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|          Frame Counter        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

**************************/

/*! Network Parameter MLE Frame Counter TLV - Tx Interface structure */
typedef struct mleTlvMleFrameCtr_tag
{
    mleTlvType_t tlvType;
    uint32_t frameCounter;
} mleTlvMleFrameCtr_t;

/*! Network Parameter MLE Frame Counter TLV - Over the Air mapping structure */
typedef struct mleOtaTlvMleFrameCtr_tag
{
    uint8_t tlvType;
    uint8_t length;
    uint8_t frameCounter[4];
} mleOtaTlvMleFrameCtr_t;

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
\fn     void MLE_TlvSrcAddrAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Source Address TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvSrcAddrAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvModeAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Mode TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvModeAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvTimeoutAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Timeout TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvTimeoutAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvChallengeAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Challenge TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvChallengeAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvResponseAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Response TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvResponseAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvLLFrameCtrAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Link Layer Frame Counter TLV to a TLV list.

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvLLFrameCtrAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvLinkQualityAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Link Quality TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvLinkQualityAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvNwkParamAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the Network Parameter TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvNwkParamAdd(void *pParams, list_t *pTlvList);

/*!*************************************************************************************************
\fn     void MLE_TlvMleFrameCtrAdd(void *pParams, list_t *pTlvList)
\brief  Function used to add the MLE Frame Counter TLV to a TLV list. 

\param  [in]    pParams         Pointer to the input TLV structure.
\param  [in]    pTlvList        Pointer to the TLV list.
***************************************************************************************************/
void MLE_TlvMleFrameCtrAdd(void *pParams, list_t *pTlvList);

#ifdef __cplusplus
}
#endif

/*================================================================================================*/
#endif  /* _MLE_BASIC_H */
