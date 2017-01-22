
#include "EmbeddedTypes.h"
#include "fsl_osa_ext.h"

#include "panic.h"
#include "memmanager.h"
#include "led.h"

#include "virtual_enet_driver.h"
#include "I2CDriver.h"
#include "i2c_abstraction.h"
#include "network_utils.h"
#include "FunctionLib.h"
#include "DataAvailablePin.h"

/*==================================================================================================
Private macros
==================================================================================================*/
#define I2C_SPEED               (390000)

#define I2C_RX_BUFF_SIZE        1600

#ifndef I2C_TASK_STACK_SIZE
    #define I2C_TASK_STACK_SIZE 512U
#endif
/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

/* I2C master */
void I2C_MasterTriggerInit();
void I2C_SetMode(i2cMode_t changeMode, void* param);

void I2C_HandleTriggerInd(void* param);
void I2C_HandleDataReq(void* param);
void I2C_HandleDataInd(void* param);

void I2C_SemWait(osaSemaphoreId_t semaphore_id);
void I2C_SemRelease(osaSemaphoreId_t semaphore_id);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
taskMsgQueue_t mI2cMsgQueue;
OSA_EXT_TASK_DEFINE(I2C_Task, OSA_PRIORITY_IDLE, 1, I2C_TASK_STACK_SIZE, 0);

/* I2C */
LDD_TDeviceData* i2cHandle;

i2cHdr_t mRecvHeader;
i2cHdrAndData_t mTransHeader;

i2cState_t mI2cState;

static uint8_t * i2cMsgPtr;
static uint32_t i2cMsgSize;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/* MASTER semaphore */
osaSemaphoreId_t i2c_sem_id;

/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\fn void I2C_SetMode(i2cMode_t changeMode, void* param)
\brief  Changes from RX/TX mode.

\param [in]  changeMode    the new mode
      
\retval      none
***************************************************************************************************/
void I2C_SetMode(i2cMode_t changeMode, void* param)
{
    LDD_TError status = ERR_BUSY;
    
    I2C_SemWait(i2c_sem_id);
    
    if(changeMode == i2cModeRx)
    {  
        /* RX */
        mI2cState = i2cStateRxHdr;
       
        while(status != ERR_OK)
        {
            status = I2CDriver_MasterReceiveBlock(i2cHandle, (uint8_t*)&mRecvHeader, sizeof(i2cHdr_t), LDD_I2C_SEND_STOP);
        }
    }
    else
    {
        /* TX */
        mI2cState = i2cStateTxHdr;
        
        i2cHdrAndData_t * pTxHeader = (i2cHdrAndData_t *)param;
        mTransHeader.OPGROUP   = pTxHeader->OPGROUP;
        mTransHeader.OPCODE    = pTxHeader->OPCODE;
        mTransHeader.LENGTH    = pTxHeader->LENGTH;
        mTransHeader.dataPtr   = pTxHeader->dataPtr;
        mTransHeader.freeMem   = pTxHeader->freeMem;

        MEM_BufferFree(param);
        
        while(status != ERR_OK)
        {
            status = I2CDriver_MasterSendBlock(i2cHandle, (uint8_t*)&mTransHeader, sizeof(i2cHdr_t), LDD_I2C_SEND_STOP);
        }
    }

}

/*!*************************************************************************************************
\fn void I2C_DataTransmitted(void* UserDataPtr)
\brief  Callback for TX done event.

\param [in]  UserDataPtr   pointer to user data(not used)
      
\retval      none
***************************************************************************************************/
void I2C_DataTransmitted(void* UserDataPtr)
{
    LDD_TError status = ERR_BUSY;
    
    if(mI2cState == i2cStateTxHdr)
    {
       //for(uint32_t i=0; i<100;i++);
       
       mI2cState = i2cStateTxPay;
       while(status != ERR_OK)
       {
          status = I2CDriver_MasterSendBlock(i2cHandle, mTransHeader.dataPtr, mTransHeader.LENGTH, LDD_I2C_SEND_STOP); 
       }
    }
    else
    {   
        //for(uint32_t i=0; i<50;i++);

        if(TRUE == mTransHeader.freeMem)
        {
            MEM_BufferFree(mTransHeader.dataPtr);
        }
        
        I2C_SemRelease(i2c_sem_id);
    }
}

/*!*************************************************************************************************
\fn void I2C_DataReceived(void* UserDataPtr)
\brief  Callback for RX receive event.

\param [in]  UserDataPtr   pointer to user data(not used)
      
\retval      none
***************************************************************************************************/
void I2C_DataReceived(void* UserDataPtr)
{
    LDD_TError status = ERR_BUSY;
       
    if(mI2cState == i2cStateRxHdr)
    {
        if(mRecvHeader.LENGTH <= I2C_RX_BUFF_SIZE)
        {       
            Led1On();
            i2cMsgSize = sizeof(i2cHdr_t) + mRecvHeader.LENGTH;
            i2cMsgPtr = MEM_BufferAlloc(i2cMsgSize);
            FLib_MemCpy(i2cMsgPtr, (void*)&mRecvHeader,sizeof(i2cHdr_t));

            mI2cState =i2cStateRxPay;

            /* enqueue payload receive */
            while(status != ERR_OK)
            {
                status = I2CDriver_MasterReceiveBlock(i2cHandle, i2cMsgPtr + sizeof(i2cHdr_t), mRecvHeader.LENGTH, LDD_I2C_SEND_STOP);
            }
        }
        else
        {   
            //bI2cError = TRUE;
        }
    }
    else
    {   
        Led1Off();
        
        /* send to I2C task */
        NWKU_SendMsg(I2C_HandleDataInd,(void*)i2cMsgPtr,&mI2cMsgQueue);
        
        mI2cState = i2cStateRxHdr;
        I2C_SemRelease(i2c_sem_id);
    }
}

/*!*************************************************************************************************
\fn void I2C_MasterTriggerInit(void* UserDataPtr)
\brief  Initiliazes the I2C trigger pin
     
\retval      none
***************************************************************************************************/
void I2C_MasterTriggerInit
(
)
{    
    DataAvailablePin_Init(NULL);
}

/*!*************************************************************************************************
\fn void I2C_MasterTriggerServiceRoutine(void* param)
\brief  Services the I2C trigger pin interrupt
     
\retval      none
***************************************************************************************************/
void I2C_MasterTriggerServiceRoutine
(
    void * param
)
{   
    NWKU_SendMsg(I2C_HandleTriggerInd, NULL, &mI2cMsgQueue);
}
/*==================================================================================================
Public functions
==================================================================================================*/

/*!*************************************************************************************************
\fn void I2C_Abstraction_Init()
\brief  Initialize function for the I2C abstraction task
     
\retval      none
***************************************************************************************************/
void I2C_Abstraction_Init()
{       
    /* I2C task queue */
    ListInit(&mI2cMsgQueue.msgQueue, 10);
    
    /* Semaphores */
    i2c_sem_id = OSA_EXT_SemaphoreCreate(1);
    
    if(!i2c_sem_id)
    {
        panic(0,(uint32_t)I2C_Abstraction_Init, 0, 0);
    }

    /* Initialize GPIO pin */
    I2C_MasterTriggerInit();

    i2cHandle = I2CDriver_Init(NULL);

    if(i2cHandle == NULL)
    {   
        panic(0,(uint32_t)I2C_Abstraction_Init, 0, 0); 
    }
    
    mI2cMsgQueue.taskEventId = OSA_EXT_EventCreate(TRUE);
    mI2cMsgQueue.taskId = OSA_EXT_TaskCreate(OSA_EXT_TASK(I2C_Task), (osaTaskParam_t)NULL);
}

/*!*************************************************************************************************
\fn void I2C_MasterSend(uint8_t *pBuffer,uint32_t length,uint8_t opGroup,uint8_t opCode)
\brief  Callback for TX done event.

\param [in]  pBuffer       pointer to user data
\param [in]  length        length of user data
\param [in]  opGroup       I2C opcode group(identifies direction Master->Slave, Slave->Master)
\param [in]  opCode        I2C opcode(identifies request fucntion)
\param [in]  freeMemory    flag for freeing or not the data memory
      
\retval      none
***************************************************************************************************/
void I2C_MasterSend
(
    uint8_t  *pBuffer, 
    uint32_t length, 
    uint8_t  opGroup, 
    uint8_t  opCode,
    bool_t   freeMemory
)
{
    i2cHdrAndData_t * pTxHeader = MEM_BufferAlloc(sizeof(i2cHdrAndData_t));

    pTxHeader->OPGROUP   = opGroup;
    pTxHeader->OPCODE    = opCode;
    pTxHeader->LENGTH    = length;
    pTxHeader->dataPtr   = pBuffer;
    pTxHeader->freeMem   = freeMemory;

    NWKU_SendMsg(I2C_HandleDataReq, (void*)pTxHeader, &mI2cMsgQueue);
}

/*!*************************************************************************************************
\fn void I2C_HandleDataReq(void* param)
\brief  Message handler for the I2C abstraction task

\param [in]  void*   pointer to message data
     
\retval      none
***************************************************************************************************/
void I2C_HandleDataReq
(
    void* param
)
{
    /* go to rx mode */
    I2C_SetMode(i2cModeTx, param);  
}

/*!*************************************************************************************************
\fn void I2C_HandleTriggerInd(void* param)
\brief  Message handler for the I2C abstraction task

\param [in]  void*   pointer to message data
     
\retval      none
***************************************************************************************************/
void I2C_HandleTriggerInd
(
    void* param
)
{
    /* go to rx mode */
    I2C_SetMode(i2cModeRx, NULL);
}

/*!*************************************************************************************************
\fn void I2C_HandleDataInd(void* param)
\brief  Message handler for the I2C abstraction task

\param [in]  void*   pointer to message data
     
\retval      none
***************************************************************************************************/
void I2C_HandleDataInd
(
    void* param
)
{

    i2cHdr_t* headerReceived = (i2cHdr_t*)param;
    if(headerReceived->OPGROUP == OPG_SLAVE_TO_MASTER)
    {
        switch(headerReceived->OPCODE)
        {
            case OPC_ENET_RECEIVE: 
            VIRTUAL_ENET_receive((uint8_t*)param, headerReceived->LENGTH + sizeof(i2cHdr_t));
            /* free the message */
            //MEM_BufferFree(param);                
            break;

            default:
            VIRTUAL_ENET_process(headerReceived, ((uint8_t*)headerReceived)+sizeof(i2cHdr_t));
            /* free the message */
            MEM_BufferFree(param);
            break;
        }
    }


}

void I2C_SemWait(osaSemaphoreId_t semaphore_id)
{
    OSA_EXT_SemaphoreWait(semaphore_id, osaWaitForever_c);
    //Led2On();
}

void I2C_SemRelease(osaSemaphoreId_t semaphore_id)
{
    OSA_EXT_SemaphorePost(semaphore_id);
    //Led2Off();
}

/*!*************************************************************************************************
\fn void I2C_Task(osaTaskParam_t argument)
\brief  I2C task
     
\retval      none
***************************************************************************************************/
void I2C_Task
(
    osaTaskParam_t argument
)
{

    while (1)
    {
        NWKU_RecvMsg(&mI2cMsgQueue);    
    }

} 
