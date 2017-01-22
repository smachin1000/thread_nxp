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

#ifndef _MLE_SECURITY_H
#define _MLE_SECURITY_H
/*!=================================================================================================
\file       mle_security.h
\brief      This is a header file for the Mesh Link Establishment security module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"

/* Network Includes */
#include "network_utils.h"

#include "mle_cfg.h"

/*==================================================================================================
Public macros
==================================================================================================*/

#if MLE_SECURITY_ENABLED

    /*! Macro for MLE security level */
    #define MLE_SECURITY_LEVEL                          5

    /*! Macro for MLE key identifier mode */
    #define MLE_KEY_ID_MODE                             1

#endif /* MLE_SECURITY_ENABLED */

/*==================================================================================================
Public type definitions
==================================================================================================*/

#if MLE_SECURITY_ENABLED
    
    /*! MLE key descriptor structure */
    typedef struct mleKeyDescriptor_tag
    {
        uint8_t     key[16];                                /*!< MLE key */
        uint32_t    outgoingFrameCounter;                   /*!< MLE outgoing frame counter */
        uint8_t     keyIndex;                               /*!< MLE key index */
        bool_t      bInUse;                                 /*!< Flag that indicates if the entry is used or not */
    }mleKeyDescriptor_t;

#endif /* MLE_SECURITY_ENABLED */


/*! Prototype of the function called by MLE to obtain extended address of a neighbor identified by IP address */
typedef bool_t (*mleNeighborGetExtendedAddr_t)(ipAddr_t* pIpAddr, uint8_t* pExtendedAddress);

/*! Prototype of the function called by MLE to obtain frameCounter of a neighbor identified by IP address */
typedef bool_t (*mleNeighborGetFrameCounter_t)(ipAddr_t* pIpAddr, uint32_t* pFrameCounter);

/*! Prototype of the function called by MLE to update frameCounter of a neighbor identified by IP address */
typedef bool_t (*mleNeighborSetFrameCounter_t)(ipAddr_t* pIpAddr, uint32_t newFrameCounter);

/*! MLE neighbor functions structure */
typedef struct mleNeighborFunctions_tag
{
    mleNeighborGetExtendedAddr_t    getExtendedAddress;     /*!< Get extended address */
    mleNeighborGetFrameCounter_t    getFrameCounter;        /*!< Get frame counter */
    mleNeighborSetFrameCounter_t    setFrameCounter;        /*!< Set frame counter */
}mleNeighborFunctions_t;

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
\fn     void MLE_RegisterNeighborFunctions(mleNeighborFunctions_t* pMleNeighborFunctions)
\brief  Public interface function for MLE security module. This function sets the callbacks used 
        by MLE security to access a neighbor table from the upper layer.

\param  [in]        pMleNeighborFunctions   Pointer to the function pointers structure.

\return             none
***************************************************************************************************/
void MLE_RegisterNeighborFunctions(mleNeighborFunctions_t* pMleNeighborFunctions);

/*!*************************************************************************************************
\private
\fn     void MLE_GetFrameCounter(void)
\brief  Gets the MLE active frame counter.

\return         uint32_t    MLE Frame Counter
***************************************************************************************************/
uint32_t MLE_GetFrameCounter(  void);
  
/*!*************************************************************************************************
\fn     uint8_t MLE_CreateKeyDescriptor(uint8_t* pKey, uint8_t keyIndex)
\brief  Public interface function for MLE security module. This function creates a key descriptor.

\param  [in]        pKey                Pointer to the MLE key.
\param  [in]        keyIndex            Key index associated with the MLE key.  

\retval             0-0xFE              Index in the mMleKeyDescriptorTable of the Key descriptor created.  
\retval             0xFF                Key descriptor was not created.
***************************************************************************************************/
uint8_t MLE_CreateKeyDescriptor(uint8_t* pKey, uint8_t keyIndex);

/*!*************************************************************************************************
\fn     uint8_t MLE_GetKeyDescriptorIndex(uint8_t keyIndex)
\brief  Public interface function for MLE security module. This function returns the index
        in the mMleKeyDescriptorTable of the key descriptor with the given key index.

\param  [in]        keyIndex                Key index associated with the MLE key.

\retval             0-0xFE              Index in the mMleKeyDescriptorTable of the Key descriptor.  
\retval             0xFF                Key descriptor not found.
***************************************************************************************************/
uint8_t MLE_GetKeyDescriptorIndex(uint8_t keyIndex);


/*!*************************************************************************************************
\fn     bool_t MLE_ActivateKeyDescriptor(uint8_t keyIndex)
\brief  Public interface function for MLE security module. This function activates a key descriptor.

\param  [in]        keyIndex            Key index associated with the MLE key.  

\retval             TRUE                Key descriptor was activated.  
\retval             FALSE               Key descriptor was not activated.
***************************************************************************************************/
bool_t MLE_ActivateKeyDescriptor(uint8_t keyIndex);


/*!*************************************************************************************************
\fn     uint8_t* MLE_MsgAlloc(bool_t bSecured, uint32_t* pMsgSize, uint8_t** ppData)
\brief  Public interface function for MLE security module. This function allocates buffer for a 
        MLE message.

\param  [in]        bSecured            Flag that indicates if the MLE message will be secured or not.  
\param  [in, out]   pMsgSize            Pointer to the MLE message size.
\param  [out]       ppData              Pointer to the address where MLE can start writing data.


\return             uint8_t*            Pointer to the new allocated buffer for MLE message.  
***************************************************************************************************/
uint8_t* MLE_MsgAlloc(bool_t bSecured, uint32_t* pMsgSize, uint8_t** ppData);


/*!*************************************************************************************************
\fn     bool_t MLE_SecureTxMessage(uint8_t* pTxMessage, uint32_t txMessageLength, 
                                                      ipAddr_t* pSrcIpAddr, ipAddr_t* pDstIpAddr, 
                                                      uint8_t* pLocalExtendedAddress, bool_t bSecured)
\brief  Public interface function for MLE security module. This function secures a MLE message to be
        transmitted, only if requested. Otherwise this function deos nothing.

\param  [in]        pTxMessage                  Pointer to the MLE message to be transmitted.
\param  [in]        txMessageLength             MLE message length.
\param  [in]        pSrcIpAddr                  Pointer to the source IP address.
\param  [in]        pDstIpAddr                  Pointer to the destination IP address.
\param  [in]        pLocalExtendedAddress       Pointer to the local extended address.
\param  [in]        bSecured                    Flag that indicates if the MLE message will be secured or not.  

\retval             TRUE                        MLE message was successfully secured.  
\retval             FALSE                       MLE message was not successfully secured. 
***************************************************************************************************/
bool_t MLE_SecureTxMessage(uint8_t* pTxMessage, uint32_t txMessageLength, ipAddr_t* pSrcIpAddr, 
                           ipAddr_t* pDstIpAddr, uint8_t* pLocalExtendedAddress, bool_t bSecured);

/*!*************************************************************************************************
\fn     bool_t MLE_UnsecureRxMessage(uint8_t* pRxMessage, uint8_t** ppRxData, uint32_t* pRxMessageLength, 
                                     ipAddr_t* pSrcIpAddr, ipAddr_t* pDstIpAddr, bool_t* pbSecured)
\brief  Public interface function for MLE security module. This function unsecures a received MLE message
        only if it is secured.

\param  [in]        pRxMessage                      Pointer to the recived MLE message.
\param  [out]       ppRxData                        Pointer to the address where starts unsecured MLE data.
\param  [in, out]   pRxMessageLength                Pointer to the MLE message size.
\param  [in]        pSrcIpAddr                      Pointer to the source IP address.
\param  [in]        pDstIpAddr                      Pointer to the destination IP address.
\param  [out]       pbSecured                       Pointer to a flag that indicates if the MLE message was 
                                                    secured or not.  

\retval             TRUE                            MLE message was successfully unsecured.  
\retval             FALSE                           MLE message was not successfully unsecured. 
***************************************************************************************************/
bool_t MLE_UnsecureRxMessage(uint8_t* pRxMessage, uint8_t** ppRxData, uint32_t* pRxMessageLength, 
                             ipAddr_t* pSrcIpAddr, ipAddr_t* pDstIpAddr, bool_t* pbSecured);
  
#ifdef __cplusplus
}
#endif

/*================================================================================================*/

#endif  /* _MLE_SECURITY_H */
