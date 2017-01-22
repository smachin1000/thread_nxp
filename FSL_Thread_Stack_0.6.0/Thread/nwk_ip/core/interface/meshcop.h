#ifndef _MESHCOP_H
#define _MESHCOP_H


/*!=================================================================================================
\file       meshcop.h
\brief      This is a header file for the MESHCOP module.

\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any
            form - including copied, transcribed, printed or by any electronic means - without
            specific written permission from Freescale.
            (c) Copyright 2014, Freescale, Inc.  All rights reserved.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "meshcop_cfg.h"
#include "network_utils.h"

#include "thread_manager.h"

#include "stack_manager_if.h"

#if MESHCOP_ENABLED


/*==================================================================================================
Public macros
==================================================================================================*/

/*==================================================================================================
Public type definitions
==================================================================================================*/
typedef struct meshCopService_tag
{
    const char *pStr;
    uint32_t len;
    void(*pfCb)(bool_t sessionStatus, void* pData, void* pSession, uint32_t pDataLen, ipAddr_t* sourceIpAddr);
} meshCopService_t;

/*!< Callback used by application to pick the Commissioner from a list of commissioner */
typedef uint8_t (*meshCopPickCommCb_t)(uint8_t *pList, uint32_t listSize);

/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Public function prototypes
==================================================================================================*/
/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void MESHCOP_Init(taskMsgQueue_t *pTaskMsgQueue, stackConfig_t *pStackCfg);

/*!*************************************************************************************************
\fn
\brief  Commissioner/No Commissioner + BorderRouter + JoinerRouter + Leader

\param  [in]
\param  [out]
\retval
***************************************************************************************************/
void MESHCOP_InitiateFirstDevice
(
    bool_t withCommissioner,
    threadInstance_t *pThreadInstance,
    meshCopPickCommCb_t pfPickComm
);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void MESHCOP_InitiateBorderRouter();

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void MESHCOP_InitiateCommissioner();

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void MESHCOP_InitiateLeader(meshCopPickCommCb_t pfPickComm);

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void MESHCOP_InitiateJoinerRouter();

/*!*************************************************************************************************
\fn
\brief
\param [in]
\param [out]
\retval
***************************************************************************************************/
void MESHCOP_InitiateJoiner(llAddr_t *pParentAddr);

/*!*************************************************************************************************
\fn     void MESHCOP_RemoveLeader()
\brief  This function should be used to disable Leader functionality on this device.

\retval none
***************************************************************************************************/
void MESHCOP_RemoveLeader();

#endif /* MESHCOP_ENABLED*/
/*================================================================================================*/
#endif
