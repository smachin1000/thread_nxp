#ifndef _I2C_ABSTRACTION_H_
#define _I2C_ABSTRACTION_H_
/*!=================================================================================================
\file       i2c_abstraction.h
\brief      This is a header file for the i2c abstraction layer. 

\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any 
            form - including copied, transcribed, printed or by any electronic means - without 
            specific written permission from Freescale.
            (c) Copyright 2013, Freescale, Inc.  All rights reserved.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "Messaging.h"
#include "fsl_osa_ext.h"

/*==================================================================================================
Public macros
==================================================================================================*/

/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef enum
{
    i2cModeRx = 0U,
    i2cModeTx
}i2cMode_t;

typedef enum
{
    i2cStateRxHdr = 0U,
    i2cStateRxPay,
    i2cStateTxHdr,
    i2cStateTxPay
}i2cState_t;

/* OPGOUPS */
typedef enum opgroups_tag
{
    OPG_MASTER_TO_SLAVE,
    OPG_SLAVE_TO_MASTER,
}VE_OPGROUP_T;

/* OPCODES */
typedef enum opcode_tag
{
    OPC_GET_MAC_ADDRESS,
    OPC_GET_ADDRESS,    
    OPC_ENET_INIT,
    OPC_ENET_GET_MTU,
    OPC_ENET_OPEN,
    OPC_ENET_CLOSE,
    OPC_ENET_SEND,
    OPC_ENET_RECEIVE,
    OPC_ENET_JOIN,
    OPC_ENET_LEAVE,
    OPC_ENET_RESET,
}VE_OPCODE_T;

typedef struct i2cHdr_tag
{
    uint8_t OPGROUP;
    uint8_t OPCODE;
    uint16_t LENGTH;
}i2cHdr_t;

typedef struct i2cHdrAndData_tag
{
    uint8_t  OPGROUP;
    uint8_t  OPCODE;
    uint16_t LENGTH;
    uint8_t* dataPtr;
    bool_t   freeMem;
}i2cHdrAndData_t;

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
\fn void I2C_Abstraction_Init()
\brief  Initialize function for the I2C abstraction task
     
\retval      none
***************************************************************************************************/
void I2C_Abstraction_Init();

/*!*************************************************************************************************
\fn void I2C_MasterSend(uint8_t *pBuffer,uint32_t length,uint8_t opGroup,uint8_t opCode)
\brief  Callback for TX done event.

\param [in]  pBuffer   pointer to user data
\param [in]  length    length of user data
\param [in]  opGroup   I2C opcode group(identifies direction Master->Slave, Slave->Master)
\param [in]  opCode    I2C opcode(identifies request fucntion)
      
\retval      none
***************************************************************************************************/
void I2C_MasterSend(uint8_t *pBuffer, uint32_t length, uint8_t opGroup, uint8_t opCode,
                            bool_t freeMemory);

void I2C_Task(osaTaskParam_t argument);
void I2C_DataReceived(void* UserDataPtr);
void I2C_DataTransmitted(void* UserDataPtr);
/*!*************************************************************************************************
\fn void I2C_MasterTriggerServiceRoutine(void* param)
\brief  Services the I2C trigger pin interrupt
     
\retval      none
***************************************************************************************************/
void I2C_MasterTriggerServiceRoutine(void * param);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*_I2C_ABSTRACTION_H_ */
