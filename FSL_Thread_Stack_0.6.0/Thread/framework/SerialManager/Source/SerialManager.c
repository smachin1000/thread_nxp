/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file SerialManager.c
* This is the source file for the Serial Manager.
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


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */

#include "SerialManager.h"
#include "Panic.h"
#include "MemManager.h"
#include "Messaging.h"
#include "FunctionLib.h"

#if (gSerialMgrUseUart_c)
  #include "UART_Adapter.h"
#endif

#if (gSerialMgrUseIIC_c)
  #include "fsl_i2c_master_driver.h"
  #include "fsl_i2c_slave_driver.h"
  #include "fsl_i2c_hal.h"
#endif

#if (gSerialMgrUseSPI_c)
  #include "SPI_Adapter.h"
#endif

#if (gSerialMgrUseUSB_c)
  #include "VirtualComInterface.h"
#endif

#include "pin_mux.h"
#include "fsl_gpio_driver.h"
#include "fsl_os_abstraction.h"
#include <string.h>


/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#ifndef gSMGR_UseOsSemForSynchronization_c
#define gSMGR_UseOsSemForSynchronization_c  (USE_RTOS)
#endif

#define mSerial_IncIdx_d(idx, max) if( ++(idx) >= (max) )  (idx) = 0;

#define mSerial_DecIdx_d(idx, max) if( (idx) > 0 )  (idx)--;     \
                              else  (idx) = (max) - 1;

#define gSMRxBufSize_c (gSerialMgrRxBufSize_c + 1)

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
/* Set the size of the Rx buffer indexes */
#if gSMRxBufSize_c < 255
typedef uint8_t bufIndex_t;
#else
typedef uint16_t bufIndex_t;
#endif
/* Defines events recognized by the SerialManager's Task */
/* Message used to enque async tx data */
typedef struct SerialManagetMsg_tag{
    pSerialCallBack_t txCallback;
    void             *pTxParam;
    uint8_t          *pData;
    uint16_t          dataSize;
}SerialMsg_t;

/* Defines the serial interface structure */
typedef struct serial_tag{
    serialInterfaceType_t  serialType;
    uint8_t                serialChannel;
    /* Rx parameters */
    bufIndex_t             rxIn;
    volatile bufIndex_t    rxOut;
    pSerialCallBack_t      rxCallback;
    void                  *pRxParam;
    uint8_t                rxBuffer[gSMRxBufSize_c];
    /* Tx parameters */
    SerialMsg_t            txQueue[gSerialMgrTxQueueSize_c];
#if gSMGR_UseOsSemForSynchronization_c
    semaphore_t            txSyncSem;
#if gSerialMgr_BlockSenderOnQueueFull_c
    semaphore_t            txQueueSem;
    uint8_t                txBlockedTasks;
#endif
#endif
    uint8_t                txIn;
    uint8_t                txOut;
    uint8_t                txCurrent;
    uint8_t                events;
    uint8_t                state;
}serial_t;

typedef enum{
    gSMGR_Rx_c     = (1<<0),
    gSMGR_TxDone_c = (1<<1),
    gSMGR_TxNew_c  = (1<<2)
}serialEventType_t;

/*
 * Driver specific data structures
 */
#if (gSerialMgrUseUart_c)
typedef struct smgrUartData_tag{
    uart_state_t state;
}smgrUartData_t;
#endif

#if (gSerialMgrUseIIC_c)
enum{
    gI2cSlaveDAP  = GPIO_MAKE_PIN(HW_GPIOC, 1U),
    gI2cMasterDAP = GPIO_MAKE_PIN(HW_GPIOC, 1U)
};

typedef struct smgrI2CSlaveData_tag{
    i2c_slave_state_t state;
}smgrI2CSlaveData_t;

typedef struct smgrI2CMasterData_tag{
  i2c_master_state_t state;
  i2c_device_t bus;
}smgrI2CMasterData_t;
#endif

#if (gSerialMgrUseSPI_c)
enum{
    gSpiSlaveDAP  = GPIO_MAKE_PIN(HW_GPIOC, 2U),
    gSpiMasterDAP = GPIO_MAKE_PIN(HW_GPIOC, 2U)
};

typedef struct smgrSPISlaveData_tag{
  spiSlaveStare_t state;
}smgrSPISlaveData_t;

typedef struct smgrSPIMasterData_tag{
  spiMasterState_t state;
  spiBusConfig_t bus;
}smgrSPIMasterData_t;
#endif

typedef union smgrDrvData_tag{
#if (gSerialMgrUseUart_c)
  smgrUartData_t uart;
#endif
#if (gSerialMgrUseIIC_c)
  smgrI2CSlaveData_t  i2cSlave;
  smgrI2CMasterData_t i2cMaster;
#endif
#if (gSerialMgrUseSPI_c)
  smgrSPISlaveData_t  spiSlave;
  smgrSPIMasterData_t spiMaster;
#endif
  void *pDrvData;
}smgrDrvData_t;

/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
#if (gSerialManagerMaxInterfaces_c)
void SerialManagerTask(task_param_t argument);
serialStatus_t Serial_WriteInternal (uint8_t InterfaceId);
uint16_t Serial_ReadInternal(uint8_t InterfaceId, uint8_t *pData, uint16_t dataSize);

void SerialManager_RxNotify(uint32_t interfaceId);
void SerialManager_TxNotify(uint32_t interfaceId);

#if (gSerialMgrUseUart_c)
static void UartTxCb(uint32_t instance);
static void UartRxCb(uint32_t instance);
#endif

#if (gSerialMgrUseSPI_c)
static void SpiSlaveTxCb(uint32_t instance);
static void SpiSlaveRxCb(uint32_t instance);
#endif

static void Serial_SyncTxCallback(void *pSer);
static void Serial_TxQueueMaintenance(serial_t *pSer);
#endif

#if defined(FWK_SMALL_RAM_CONFIG)
void FwkInit(void);
#endif

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
extern const uint8_t gUseRtos_c;

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */
#if gSerialManagerMaxInterfaces_c

#if defined(FWK_SMALL_RAM_CONFIG)
extern event_t  gFwkCommonEvent;
extern task_handler_t gFwkCommonTaskId;
#define gSerialManagerTaskId gFwkCommonTaskId
#define mSMTaskEvent gFwkCommonEvent

#else

OSA_TASK_DEFINE( SerialManagerTask, gSerialTaskStackSize_c );
#if defined(FSL_RTOS_MQX)
  uint8_t SMGR_Task_stack[gSerialTaskStackSize_c];
  #define gSmgrTaskStack_d (uint32_t*)SMGR_Task_stack
#else
  #define gSmgrTaskStack_d NULL
#endif
  
task_handler_t gSerialManagerTaskId;
event_t        mSMTaskEvent;
#endif /* defined(FWK_SMALL_RAM_CONFIG) */

serial_t       mSerials[gSerialManagerMaxInterfaces_c];
static smgrDrvData_t mDrvData[gSerialManagerMaxInterfaces_c];

/*
 * Data Available Pin configurations for IIC/SPI drivers
 */
#if (gSerialMgrUseIIC_c)
uint8_t mSmgrI2cSlaveChannel; //gcapraru: hack
const gpio_output_pin_user_config_t gDapI2CSlaveCfg = {
    .pinName = gI2cSlaveDAP,
    .config.outputLogic = 0,
    .config.slewRate = kPortFastSlewRate,
#if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
    .config.isOpenDrainEnabled = FALSE,
#endif
    .config.driveStrength = kPortLowDriveStrength,
};

const gpio_input_pin_user_config_t gDapI2CMasterCfg = {
    .pinName = gI2cMasterDAP,
    .config.isPullEnable = FALSE,
    .config.pullSelect = kPortPullDown,
    .config.isPassiveFilterEnabled = FALSE,
    .config.interrupt = kPortIntRisingEdge
};
#endif

#if gSerialMgrUseSPI_c
const gpio_output_pin_user_config_t gDapSpiSlaveCfg = {
    .pinName = gSpiSlaveDAP,
    .config.outputLogic = 0,
    .config.slewRate = kPortFastSlewRate,
#if FSL_FEATURE_PORT_HAS_OPEN_DRAIN
    .config.isOpenDrainEnabled = FALSE,
#endif
    .config.driveStrength = kPortLowDriveStrength,
};

const gpio_input_pin_user_config_t gDapSpiMasterCfg = {
    .pinName = gSpiMasterDAP,
    .config.isPullEnable = FALSE,
    .config.pullSelect = kPortPullDown,
    .config.isPassiveFilterEnabled = FALSE,
    .config.interrupt = kPortIntRisingEdge
};

const gpio_input_pin_user_config_t gDapSPIMasterCfg = {
    .pinName = gSpiMasterDAP,
    .config.isPullEnable = FALSE,
    .config.pullSelect = kPortPullDown,
    .config.isPassiveFilterEnabled = FALSE,
    .config.interrupt = kPortIntRisingEdge
};
#endif

#endif /* #if gSerialManagerMaxInterfaces_c */

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
* \brief   Creates the SerialManager's task and initializes internal data structures
*
********************************************************************************** */
void SerialManager_Init( void )
{
#if (gSerialManagerMaxInterfaces_c)       
    static uint8_t initialized = FALSE;

    /* Check if SMGR is already initialized */
    if( initialized )
        return;

    initialized = TRUE;

    /* Fill the structure with zeros */
    FLib_MemSet( mSerials, 0x00, sizeof(mSerials) );
#if defined(FWK_SMALL_RAM_CONFIG)
    FwkInit();
#else
    osa_status_t status;
    
    status = OSA_EventCreate( &mSMTaskEvent, kEventAutoClear);
    if( kStatus_OSA_Success != status )
    {
        panic(0,0,0,0);
        return;
    }
    status = OSA_TaskCreate(SerialManagerTask, "SMGR_Task", gSerialTaskStackSize_c, gSmgrTaskStack_d,
                            gSerialTaskPriority_c, (task_param_t)NULL, FALSE, &gSerialManagerTaskId);
    if( kStatus_OSA_Success != status )
    {
        panic(0,0,0,0);
        return;
    }
#endif /* #if defined(FWK_SMALL_RAM_CONFIG) */
#endif /* #if (gSerialManagerMaxInterfaces_c) */
}

/*! *********************************************************************************
* \brief   The main task of the Serial Manager
*
* \param[in] initialData unused
*
********************************************************************************** */
#if (gSerialManagerMaxInterfaces_c)
void SerialManagerTask(task_param_t argument)
{
    uint16_t i;
    uint8_t ev;    

#if defined(FWK_SMALL_RAM_CONFIG)
    {
#else
    event_flags_t  mSMTaskEventFlags;        

    while( 1 )
    {
        /* Wait for an event. The task will block here. */
        (void)OSA_EventWait(&mSMTaskEvent, 0x00FFFFFF, FALSE, OSA_WAIT_FOREVER ,&mSMTaskEventFlags);
#endif
        for( i = 0; i < gSerialManagerMaxInterfaces_c; i++ )
        {
            OSA_EnterCritical(kCriticalDisableInt);
            ev = mSerials[i].events;
            mSerials[i].events = 0;
            OSA_ExitCritical(kCriticalDisableInt);

            if ( (ev & gSMGR_Rx_c) &&
                 (NULL != mSerials[i].rxCallback) )
            {
                mSerials[i].rxCallback( mSerials[i].pRxParam );
            }

            if( ev & gSMGR_TxDone_c )
            {
                Serial_TxQueueMaintenance(&mSerials[i]);
            }

            /* If the Serial is IDLE and there is data to tx */
            OSA_EnterCritical(kCriticalDisableInt);
            if( mSerials[i].state == 0 && 
                mSerials[i].txQueue[mSerials[i].txCurrent].dataSize )
            {
                if ( gSerial_Success_c != Serial_WriteInternal( i ) )
                {
                    SerialManager_TxNotify( i );
                }
                OSA_ExitCritical(kCriticalDisableInt);
            }
            else
            {
                OSA_ExitCritical(kCriticalDisableInt);
            }
        }
         
#if !defined(FWK_SMALL_RAM_CONFIG)    
        /* For BareMetal break the while(1) after 1 run */
        if (gUseRtos_c == 0)
        {
            break;
        }
#endif
    } /* while(1) */
}
#endif

/*! *********************************************************************************
* \brief   Initialize a communication interface.
*
* \param[in] pInterfaceId pointer to a location where the interface Id will be stored
* \param[in] interfaceType the tupe of the interface: UART/SPI/IIC/USB
* \param[in] channel the number of the HW module (ex: if UART1 is used, this value should be 1)
*
* \return The interface number if success or gSerialManagerInvalidInterface_c if an error occured.
*
********************************************************************************** */
serialStatus_t Serial_InitInterface( uint8_t *pInterfaceId,
                                     serialInterfaceType_t interfaceType,
                                     uint8_t channel )
{
#if gSerialManagerMaxInterfaces_c
    uint8_t i;

    *pInterfaceId = gSerialMgrInvalidIdx_c;

    for ( i=0; i<gSerialManagerMaxInterfaces_c; i++ )
    {
        if ( (mSerials[i].serialType == interfaceType) &&
            (mSerials[i].serialChannel == channel) )
        {
            /* The Interface is allready opened. */
            return gSerial_InterfaceInUse_c;
        }
        else if ( mSerials[i].serialType == gSerialMgrNone_c )
        {
            mSerials[i].serialChannel = channel;
            switch ( interfaceType )
            {
#if (gSerialMgrUseUart_c)
            case gSerialMgrUart_c:
                Uart_Init(channel, &(mDrvData[i].uart.state), UartTxCb, UartRxCb);
                /* init Rx process */
                Uart_ReceiveData(channel, &mSerials[i].rxBuffer[mSerials[i].rxIn], 1 );
                break;
#endif
#if gSerialMgrUseUSB_c
            case gSerialMgrUSB_c:
                mDrvData[i].pDrvData = VirtualCom_Init(i);
                if (NULL == mDrvData[i].pDrvData)
                {
                    return gSerial_InternalError_c;
                }
                break;
#endif
#if gSerialMgrUseIIC_c
            case gSerialMgrIICMaster_c:
                mDrvData[i].i2cMaster.bus.address = gSerialMgrIICAddress_c;
                mDrvData[i].i2cMaster.bus.baudRate_kbps = 50;
                configure_i2c_pins(channel);
                I2C_DRV_MasterInit(channel, &(mDrvData[i].i2cMaster.state));
                
                GPIO_DRV_InputPinInit(&gDapI2CMasterCfg);
                //gcapraru: OSA_InstallIntHandler(g_portIrqId[gI2cMasterDAP], ???);
                break;                

            case gSerialMgrIICSlave_c:
                mSmgrI2cSlaveChannel = channel;
                configure_i2c_pins(channel);
                I2C_DRV_SlaveInit(channel, gSerialMgrIICAddress_c, &(mDrvData[i].i2cSlave.state));
                I2C_DRV_SlaveReceiveData(channel, &mSerials[i].rxBuffer[mSerials[i].rxIn], 1 );
                GPIO_DRV_OutputPinInit(&gDapI2CSlaveCfg);
                break;
#endif
#if gSerialMgrUseSPI_c
            case gSerialMgrSPIMaster_c:
                mDrvData[i].spiMaster.bus.bitsPerSec = gSPI_BaudRate_100000_c;
                mDrvData[i].spiMaster.bus.bitsPerFrame = 8;
                mDrvData[i].spiMaster.bus.clkPhase = gSpiClkPhase_FirstEdge_d;
                mDrvData[i].spiMaster.bus.clkPolarity = gSpiClk_ActiveHigh_d;
                mDrvData[i].spiMaster.bus.direction = gSpiMsbFirst_d;

                SpiMaster_Init(channel, &(mDrvData[i].spiMaster.state));
                SpiMaster_Configure(channel, &(mDrvData[i].spiMaster.bus));
                GPIO_DRV_InputPinInit(&gDapSpiMasterCfg);
                //gcapraru: OSA_InstallIntHandler(g_portIrqId[gSpiMasterDAP], ???);
                break;

            case gSerialMgrSPISlave_c:
                SpiSlave_Init(channel, &(mDrvData[i].spiSlave.state), SpiSlaveRxCb, SpiSlaveTxCb);
                GPIO_DRV_OutputPinInit(&gDapSpiSlaveCfg);
                break;
#endif
            default:
                return gSerial_InvalidInterface_c;
            }
#if gSMGR_UseOsSemForSynchronization_c
            if( kStatus_OSA_Success != OSA_SemaCreate(&mSerials[i].txSyncSem, 0) )
            {
                return gSerial_SemCreateError_c;
            }
            
            
#if gSerialMgr_BlockSenderOnQueueFull_c
            if( kStatus_OSA_Success != OSA_SemaCreate(&mSerials[i].txQueueSem, 0) )
            {
                return gSerial_SemCreateError_c;
            }
#endif /* gSerialMgr_BlockSenderOnQueueFull_c */
#endif /* gSMGR_UseOsSemForSynchronization_c */

            mSerials[i].serialType = interfaceType;
            *pInterfaceId = i;
            return gSerial_Success_c;
        }
    }

    /* There are no more free interfaces. */
    return gSerial_MaxInterfacesReached_c;
#else
    (void)interfaceType;
    (void)channel;
    (void)pInterfaceId;
    return gSerial_Success_c;
#endif
}

/*! *********************************************************************************
* \brief   Transmit a data buffer asynchronously
*
* \param[in] InterfaceId the interface number
* \param[in] pBuf pointer to data location
* \param[in] bufLen the number of bytes to be sent
* \param[in] pSerialRxCallBack pointer to a function that will be called when
*            a new char is available
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_AsyncWrite( uint8_t id,
                                  uint8_t *pBuf,
                                  uint16_t bufLen,
                                  pSerialCallBack_t cb,
                                  void *pTxParam )
{
#if gSerialManagerMaxInterfaces_c
    SerialMsg_t *pMsg = NULL;
    serial_t *pSer = &mSerials[id];

#if gSerialMgr_ParamValidation_d
    if( (NULL == pBuf) || (0 == bufLen)       ||
        (id >= gSerialManagerMaxInterfaces_c) ||
        (pSer->serialType == gSerialMgrNone_c) )
    {
        return gSerial_InvalidParameter_c;
    }
#endif

  if( (pSer->txQueue[pSer->txIn].dataSize) || (pSer->txQueue[pSer->txIn].txCallback) )
    {
#if gSerialMgr_BlockSenderOnQueueFull_c
#if gSMGR_UseOsSemForSynchronization_c
        if( OSA_TaskGetHandler() != gSerialManagerTaskId )
        {
            OSA_EnterCritical(kCriticalDisableInt);
            pSer->txBlockedTasks++;
            OSA_ExitCritical(kCriticalDisableInt);
            (void)OSA_SemaWait(&pSer->txQueueSem, OSA_WAIT_FOREVER);
        }
        else
#endif /* gSMGR_UseOsSemForSynchronization_c */
        {
          while( (pSer->txQueue[pSer->txIn].dataSize) || (pSer->txQueue[pSer->txIn].txCallback) )
                Serial_TxQueueMaintenance(pSer);
        }
#else
        if( OSA_TaskGetHandler() == gSerialManagerTaskId )
        {
            Serial_TxQueueMaintenance(pSer);
        }
#endif
    }  

    /* Check if slot is free */
    OSA_EnterCritical(kCriticalDisableInt);
  if( (0 == pSer->txQueue[pSer->txIn].dataSize) && (NULL == pSer->txQueue[pSer->txIn].txCallback) )
    {
        pMsg = &pSer->txQueue[pSer->txIn];
        pMsg->dataSize   = bufLen;
        pMsg->pData      = (void*)pBuf;
        pMsg->txCallback = cb;
        pMsg->pTxParam   = pTxParam;
        mSerial_IncIdx_d(pSer->txIn, gSerialMgrTxQueueSize_c);
    }

    if( pMsg )
    {
        /* If interface is Idle, start Tx asap */
        if( pSer->state == 0 )
        {
            if ( gSerial_Success_c != Serial_WriteInternal( id ) )
            {
                pSer->state = 0;
                SerialManager_TxNotify( id );
            }
        }
        else
        {
            (void)OSA_EventSet(&mSMTaskEvent, gSMGR_TxNew_c);
        }

        OSA_ExitCritical(kCriticalDisableInt);
        return gSerial_Success_c;
    }
    OSA_ExitCritical(kCriticalDisableInt);
    
    return gSerial_OutOfMemory_c;
#else
    (void)id;
    (void)pBuf;
    (void)bufLen;
    (void)cb;
    (void)pTxParam;
    return gSerial_Success_c;
#endif /* gSerialManagerMaxInterfaces_c */
}


/*! *********************************************************************************
* \brief Transmit a data buffer synchronously. The task will block until the Tx is done
*
* \param[in] pBuf pointer to data location
* \param[in] bufLen the number of bytes to be sent
* \param[in] InterfaceId the interface number
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_SyncWrite( uint8_t InterfaceId,
                                 uint8_t *pBuf,
                                 uint16_t bufLen )
{
    serialStatus_t status = gSerial_Success_c;
#if gSerialManagerMaxInterfaces_c
    pSerialCallBack_t cb = NULL;
    volatile serial_t *pSer = &mSerials[InterfaceId];

#if gSMGR_UseOsSemForSynchronization_c
    /* If the calling task is SMGR do not block on semaphore */
    if( OSA_TaskGetHandler() != gSerialManagerTaskId )
         cb = Serial_SyncTxCallback;
#endif

    status  = Serial_AsyncWrite(InterfaceId, pBuf, bufLen, cb, (void*)pSer);

    if( gSerial_Success_c == status )
    {
        /* Wait until Tx finishes. The sem will be released by the SMGR task */
#if gSMGR_UseOsSemForSynchronization_c
        if( cb )
        {
            (void)OSA_SemaWait((semaphore_t*)&pSer->txSyncSem, OSA_WAIT_FOREVER);
        }
        else
#endif
        {
            while(pSer->state);
        }
    }
#else
    (void)pBuf;
    (void)bufLen;
    (void)InterfaceId;
#endif /* gSerialManagerMaxInterfaces_c */
    return status;
}

/*! *********************************************************************************
* \brief   Returns a specified number of characters from the Rx buffer
*
* \param[in] InterfaceId the interface number
* \param[out] pData pointer to location where to store the characters
* \param[in] dataSize the number of characters to be read
* \param[out] bytesRead the number of characters read
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_Read( uint8_t InterfaceId,
                            uint8_t *pData,
                            uint16_t dataSize,
                            uint16_t *bytesRead )
{
#if (gSerialManagerMaxInterfaces_c)
    uint32_t status = gSerial_Success_c;

#if gSerialMgr_ParamValidation_d
    if ( (InterfaceId >= gSerialManagerMaxInterfaces_c) ||
        (NULL == pData) || (NULL == bytesRead) || (0 == dataSize) )
        return gSerial_InvalidParameter_c;
#endif

    switch ( mSerials[InterfaceId].serialType )
    {
#if gSerialMgrUseUart_c
    case gSerialMgrUart_c:
        *bytesRead = Serial_ReadInternal( InterfaceId, pData, dataSize );
        break;
#endif

#if gSerialMgrUseUSB_c
    case gSerialMgrUSB_c:
        *bytesRead = Serial_ReadInternal( InterfaceId, pData, dataSize );
        VirtualCom_SMReadNotify( mDrvData[InterfaceId].pDrvData );
        break;
#endif

#if gSerialMgrUseIIC_c
    case gSerialMgrIICMaster_c:
        status = I2C_DRV_MasterReceiveDataBlocking(mSerials[InterfaceId].serialChannel,
                                                   NULL, NULL, 0,
                                                   pData, dataSize, OSA_WAIT_FOREVER);
        if( status != kStatus_I2C_Success )
            return gSerial_InternalError_c;
        
        *bytesRead = dataSize;
        break;

    case gSerialMgrIICSlave_c:
        *bytesRead = Serial_ReadInternal( InterfaceId, pData, dataSize );
        break;
#endif

#if gSerialMgrUseSPI_c
    case gSerialMgrSPISlave_c:
        *bytesRead = Serial_ReadInternal( InterfaceId, pData, dataSize );
        break;

    case gSerialMgrSPIMaster_c:
        SpiMaster_SyncTransfer(mSerials[InterfaceId].serialChannel, 
                               NULL, pData, dataSize);
        break;
#endif
    default:
        return (serialStatus_t)status;
    }
    
    return gSerial_Success_c;
#else
    (void)InterfaceId;
    (void)pData;
    (void)dataSize;
    (void)bytesRead;
    return gSerial_InvalidInterface_c;
#endif
}

/*! *********************************************************************************
* \brief   Returns a the number of bytes available in the RX buffer
*
* \param[in] InterfaceId the interface number
* \param[out] bytesCount the number of bytes available
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_RxBufferByteCount( uint8_t InterfaceId, uint16_t *bytesCount )
{
#if (gSerialManagerMaxInterfaces_c)
#if gSerialMgr_ParamValidation_d
    if ( (InterfaceId >= gSerialManagerMaxInterfaces_c) ||
        (NULL == bytesCount) )
        return  gSerial_InvalidParameter_c;
#endif
    OSA_EnterCritical(kCriticalDisableInt);
    if( mSerials[InterfaceId].rxIn >= mSerials[InterfaceId].rxOut )
    {
        *bytesCount = mSerials[InterfaceId].rxIn - mSerials[InterfaceId].rxOut;
    }
    else
    {
        *bytesCount = gSMRxBufSize_c - mSerials[InterfaceId].rxOut + mSerials[InterfaceId].rxIn;
    }
    OSA_ExitCritical(kCriticalDisableInt);
#else
    (void)bytesCount;
    (void)InterfaceId;
#endif
    return gSerial_Success_c;
}

/*! *********************************************************************************
* \brief   Sets a pointer to a function that will be called when data is received
*
* \param[in] InterfaceId the interface number
* \param[in] pfCallBack pointer to the function to be called
* \param[in] pRxParam pointer to a parameter which will be passed to the CB function
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_SetRxCallBack( uint8_t InterfaceId, pSerialCallBack_t cb, void *pRxParam )
{
#if (gSerialManagerMaxInterfaces_c)
#if gSerialMgr_ParamValidation_d
    if ( InterfaceId >= gSerialManagerMaxInterfaces_c )
        return gSerial_InvalidParameter_c;
#endif
    mSerials[InterfaceId].rxCallback = cb;
    mSerials[InterfaceId].pRxParam = pRxParam;
#else
    (void)InterfaceId;
    (void)cb;
    (void)pRxParam;
#endif
    return gSerial_Success_c;
}

/*! *********************************************************************************
* \brief   Set the communication speed for an interface
*
* \param[in] baudRate communication speed
* \param[in] InterfaceId the interface number
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_SetBaudRate( uint8_t InterfaceId, uint32_t baudRate  )
{
#if gSerialManagerMaxInterfaces_c

#if gSerialMgr_ParamValidation_d
    if ( (InterfaceId >= gSerialManagerMaxInterfaces_c) ||
        (0 == baudRate) )
        return gSerial_InvalidParameter_c;
#endif

    switch ( mSerials[InterfaceId].serialType )
    {
#if (gSerialMgrUseUart_c)
    case gSerialMgrUart_c:
        Uart_SetBaudrate(mSerials[InterfaceId].serialChannel, baudRate);
        break;
#endif
#if gSerialMgrUseIIC_c
    case gSerialMgrIICMaster_c:
        mDrvData[InterfaceId].i2cMaster.bus.baudRate_kbps = baudRate/1000;
        break;
    case gSerialMgrIICSlave_c:
        return gSerial_InvalidInterface_c;
        break;
#endif
#if gSerialMgrUseSPI_c
    case gSerialMgrSPIMaster_c:
        mDrvData[InterfaceId].spiMaster.bus.bitsPerSec = baudRate;
        SpiMaster_Configure(mSerials[InterfaceId].serialChannel,
                                    &mDrvData[InterfaceId].spiMaster.bus);
        break;
    case gSerialMgrSPISlave_c:
        return gSerial_InvalidInterface_c;
        break;
#endif
#if gSerialMgrUseUSB_c
    case gSerialMgrUSB_c:
        /* Nothing to do here. */
        break;
#endif
    default:
        return gSerial_InvalidInterface_c;
    }
#endif
    return gSerial_Success_c;
}

/*! *********************************************************************************
* \brief   Prints a string to the serial interface
*
* \param[in] InterfaceId the interface number
* \param[in] pString pointer to the string to be printed
* \param[in] allowToBlock specify if the task will wait for the tx to finish or not.
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_Print( uint8_t InterfaceId, char* pString, serialBlock_t allowToBlock )
{
#if gSerialManagerMaxInterfaces_c
    if ( allowToBlock )
    {
        return Serial_SyncWrite( InterfaceId, (uint8_t*)pString, strlen(pString) );
    }
    else
    {
        return Serial_AsyncWrite( InterfaceId, (uint8_t*)pString, strlen(pString), NULL, NULL );
    }
#else
    (void)pString;
    (void)allowToBlock;
    (void)InterfaceId;
    return gSerial_Success_c;
#endif
}

/*! *********************************************************************************
* \brief   Prints an number in hedadecimal format to the serial interface
*
* \param[in] InterfaceId the interface number
* \param[in] hex pointer to the number to be printed
* \param[in] len the number ob bytes of the number
* \param[in] flags specify display options: comma, space, new line
*
* \return The status of the operation
*
* \remarks The task will waituntil the tx has finished
*
********************************************************************************** */
serialStatus_t Serial_PrintHex( uint8_t InterfaceId,
                                uint8_t *hex,
                                uint8_t len,
                                uint8_t flags )
{
#if (gSerialManagerMaxInterfaces_c)
    uint8_t i=0;
    serialStatus_t status;
    uint8_t hexString[6]; /* 2 bytes  - hexadecimal display
    1 byte   - separator ( comma)
    1 byte   - separator ( space)
    2 bytes  - new line (\n\r)  */

    if ( !(flags & gPrtHexBigEndian_c) )
        hex = hex + (len-1);

    while ( len )
    {
        /* start preparing the print of a new byte */
        i=0;
        hexString[i++] = HexToAscii( (*hex)>>4 );
        hexString[i++] = HexToAscii( *hex );

        if ( flags & gPrtHexCommas_c )
        {
            hexString[i++] = ',';
        }
        if ( flags & gPrtHexSpaces_c )
        {
            hexString[i++] = ' ';
        }
        hex = hex + (flags & gPrtHexBigEndian_c ? 1 : -1);
        len--;

        if ( (len == 0) && (flags & gPrtHexNewLine_c) )
        {
            hexString[i++] = '\n';
            hexString[i++] = '\r';
        }

        /* transmit formatted byte */
        status = Serial_SyncWrite( InterfaceId, (uint8_t*)hexString, (uint8_t)i) ;
        if ( gSerial_Success_c != status )
            return status;
    }
#else
    /* Avoid compiler warning */
    (void)hex;
    (void)len;
    (void)InterfaceId;
    (void)flags;
#endif
    return gSerial_Success_c;
}

/*! *********************************************************************************
* \brief   Prints an unsigned integer to the serial interface
*
* \param[in] InterfaceId the interface number
* \param[in] nr the number to be printed
*
* \return The status of the operation
*
* \remarks The task will waituntil the tx has finished
*
********************************************************************************** */
serialStatus_t Serial_PrintDec( uint8_t InterfaceId, uint32_t nr )
{
#if (gSerialManagerMaxInterfaces_c)
#define gDecStringLen_d 12
    uint8_t i = gDecStringLen_d-1;
    uint8_t decString[gDecStringLen_d];

    if ( nr == 0 )
    {
        decString[i] = '0';
    }
    else
    {
        while ( nr )
        {
            decString[i] = '0' + (uint8_t)(nr % 10);
            nr = nr / 10;
            i--;
        }
        i++;
    }

    /* transmit formatted byte */
    return Serial_SyncWrite( InterfaceId, (uint8_t*)&decString[i], gDecStringLen_d-i );
#else
    (void)nr;
    (void)InterfaceId;
    return gSerial_Success_c;
#endif
}


/*! *********************************************************************************
* \brief   Configures the enabled hardware modules of the given interface type as a wakeup source from STOP mode  
*
* \param[in] interface type of the modules to configure
*
* \return  gSerial_Success_c if there is at least one module to configure
*          gSerial_InvalidInterface_c otherwise 
* \pre
*
* \post
*
* \remarks 
*
********************************************************************************** */

serialStatus_t Serial_EnableLowPowerWakeup( serialInterfaceType_t interfaceType )
{
#if gSerialManagerMaxInterfaces_c
    if(interfaceType == gSerialMgrUart_c)
    {
#if (gSerialMgrUseUart_c)
        Uart_EnableLowPowerWakeup();
        return gSerial_Success_c; 
#endif
    }
#else
    (void)interfaceType;
#endif
    return gSerial_InvalidInterface_c;
}

/*! *********************************************************************************
* \brief   Configures the enabled hardware modules of the given interface type as modules without wakeup capabilities  
*
* \param[in] interface type of the modules to configure
*
* \return  gSerial_Success_c if there is at least one module to configure 
*          gSerial_InvalidInterface_c otherwise 
* \pre
*
* \post
*
* \remarks 
*
********************************************************************************** */

serialStatus_t Serial_DisableLowPowerWakeup( serialInterfaceType_t interfaceType )
{
#if gSerialManagerMaxInterfaces_c
    if(interfaceType == gSerialMgrUart_c)
    {
#if (gSerialMgrUseUart_c)
        Uart_DisableLowPowerWakeup();
        return gSerial_Success_c; 
#endif
    }
#else
    (void)interfaceType;
#endif
    return gSerial_InvalidInterface_c;
}

/*! *********************************************************************************
* \brief   Decides whether a enabled hardware module of the given interface type woke up the CPU from STOP mode.  
*
* \param[in] interface type of the modules to be evaluated as wakeup source.
*
* \return  TRUE if a module of the given interface type was the wakeup source
*          FALSE otherwise 
* \pre
*
* \post
*
* \remarks 
*
********************************************************************************** */

bool_t Serial_IsWakeUpSource( serialInterfaceType_t interfaceType)
{
#if gSerialManagerMaxInterfaces_c
    if(interfaceType == gSerialMgrUart_c)
    {
#if (gSerialMgrUseUart_c)
        return Uart_IsWakeUpSource();
#endif
    }
#else
    (void)interfaceType;
#endif
    return FALSE;
}


/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************* */
#if (gSerialManagerMaxInterfaces_c)
/*! *********************************************************************************
* \brief Transmit a data buffer to the specified interface.
*
* \param[in] InterfaceId the interface number
*
* \return The status of the operation
*
********************************************************************************** */
serialStatus_t Serial_WriteInternal( uint8_t InterfaceId )
{
    serial_t *pSer = &mSerials[InterfaceId];
    uint16_t idx = pSer->txCurrent;
    uint32_t status = 0;

    (void)status;

    switch ( mSerials[InterfaceId].serialType )
    {
#if (gSerialMgrUseUart_c)
    case gSerialMgrUart_c:
        Uart_SendData( pSer->serialChannel, 
                       pSer->txQueue[idx].pData, 
                       pSer->txQueue[idx].dataSize );
        break;
#endif

#if gSerialMgrUseUSB_c
    case gSerialMgrUSB_c:
        status = VirtualCom_Write( mDrvData[InterfaceId].pDrvData, 
                                pSer->txQueue[idx].pData, 
                                pSer->txQueue[idx].dataSize );
        break;
#endif

#if gSerialMgrUseIIC_c
    case gSerialMgrIICMaster_c:
        status = I2C_DRV_MasterSendDataBlocking( pSer->serialChannel, 
                                                 NULL, NULL, 0,
                                                 pSer->txQueue[idx].pData, 
                                                 pSer->txQueue[idx].dataSize, 
                                                 OSA_WAIT_FOREVER );

        if( status != kStatus_I2C_Success  )
        {
            pSer->state = 0;
            return gSerial_InternalError_c;
        }

        break;

    case gSerialMgrIICSlave_c:
        /* Notify IIC Master that we have data to send */
        I2C_DRV_SlaveSendData(pSer->serialChannel, 
                              pSer->txQueue[idx].pData, 
                              pSer->txQueue[idx].dataSize);
        GPIO_DRV_SetPinOutput(gI2cSlaveDAP);
        break;
#endif

#if gSerialMgrUseSPI_c
    case gSerialMgrSPISlave_c:
        /* Notify SPI Master that we have data to send */
        GPIO_DRV_SetPinOutput(gSpiSlaveDAP);
        break;

    case gSerialMgrSPIMaster_c:
        SpiMaster_SyncTransfer(pSer->serialChannel,
                               pSer->txQueue[idx].pData,
                               NULL,
                               pSer->txQueue[idx].dataSize);
        break;
#endif
    default:
        return gSerial_InvalidInterface_c;
    }
    if (status == gSerial_Success_c)
    {
        pSer->state = 1;
        return gSerial_Success_c;
    }
    else
    {
        return gSerial_InternalError_c;
    }
}

/*! *********************************************************************************
* \brief Inform the Serial Manager task that new data is available
*
* \param[in] pData The id interface
*
* \return none
*
* \remarks Called from usb task
*
********************************************************************************** */

#if gSerialMgrUseUSB_c
void SerialManager_VirtualComRxNotify(uint8_t* pData, uint16_t dataSize, uint8_t interface)
{

  while(dataSize)
  {
    mSerials[interface].rxBuffer[mSerials[interface].rxIn] = *pData++;
    mSerial_IncIdx_d(mSerials[interface].rxIn, gSMRxBufSize_c);
    OSA_EnterCritical(kCriticalDisableInt);
    if(mSerials[interface].rxIn == mSerials[interface].rxOut)
    {
      mSerial_IncIdx_d(mSerials[interface].rxOut, gSMRxBufSize_c);
    }
    OSA_ExitCritical(kCriticalDisableInt);
    dataSize--;
  }
  
   mSerials[interface].events |= gSMGR_Rx_c;
   (void)OSA_EventSet(&mSMTaskEvent, gSMGR_Rx_c);    
  
}
#endif
/*! *********************************************************************************
* \brief Inform the Serial Manager task that new data is available
*
* \param[in] pData The id interface
*
* \return none
*
* \remarks Called from ISR
*
********************************************************************************** */
void SerialManager_RxNotify( uint32_t i )
{
    mSerial_IncIdx_d(mSerials[i].rxIn, gSMRxBufSize_c);
    if(mSerials[i].rxIn == mSerials[i].rxOut)
    {
        mSerial_IncIdx_d(mSerials[i].rxOut, gSMRxBufSize_c);
    }

    switch( mSerials[i].serialType )
    {
#if (gSerialMgrUseUart_c)
    case gSerialMgrUart_c:
        Uart_ReceiveData(mSerials[i].serialChannel, &mSerials[i].rxBuffer[mSerials[i].rxIn], 1);
        break;
#endif

#if gSerialMgrUseSPI_c
    case gSerialMgrSPISlave_c:
        break;
    case gSerialMgrSPIMaster_c:
        break;
#endif

#if gSerialMgrUseIIC_c
    case gSerialMgrIICSlave_c:
        I2C_DRV_SlaveReceiveData(channel, &mSerials[i].rxBuffer[mSerials[i].rxIn], 1 );
        break;
    case gSerialMgrIICMaster_c:
        break;
#endif
    default:
        break;
    }

    mSerials[i].events |= gSMGR_Rx_c;
   (void)OSA_EventSet(&mSMTaskEvent, gSMGR_Rx_c);
}

/*! *********************************************************************************
* \brief Inform the Serial Manager task that a transmission has finished
*
* \param[in] pData the Id interface
*
* \return none
*
* \remarks Called from ISR
*
********************************************************************************** */
void SerialManager_TxNotify( uint32_t i )
{
    serial_t *pSer = &mSerials[i];

    pSer->events |= gSMGR_TxDone_c;
    pSer->txQueue[pSer->txCurrent].dataSize = 0; //Mark as transmitted
    mSerial_IncIdx_d(pSer->txCurrent, gSerialMgrTxQueueSize_c);

    /* Transmit next block if available */
    if( pSer->txCurrent != pSer->txIn )
    {
        if( gSerial_Success_c != Serial_WriteInternal(i) )
        {
            pSer->state = 0;
        }
    }
    else
    {
        pSer->state = 0;
#if (gSerialMgrUseIIC_c)
        if( pSer->serialType == gSerialMgrIICSlave_c )
        {
            GPIO_DRV_ClearPinOutput(gI2cSlaveDAP);
        } 
#endif
#if (gSerialMgrUseSPI_c)
        if( pSer->serialType == gSerialMgrSPISlave_c )
        {
            GPIO_DRV_ClearPinOutput(gSpiSlaveDAP);
        }
#endif
    }
    (void)OSA_EventSet(&mSMTaskEvent, gSMGR_TxDone_c);
}

/*! *********************************************************************************
* \brief Retrieve Rx data from internal SM buffer
*
* \param[in] InterfaceId the interface number
* \param[out] pData pointer to location where to store the characters
* \param[in] dataSize the number of characters to be read
*
* \return the number of characters retrieved
*
********************************************************************************** */
uint16_t Serial_ReadInternal( uint8_t InterfaceId,
                             uint8_t *pData,
                             uint16_t dataSize )
{
    uint16_t bytes;
    serial_t *pSer = &mSerials[InterfaceId];
    
    Serial_RxBufferByteCount(InterfaceId, &bytes);

    if( bytes > 0 )
    {
        uint16_t i;

        if( bytes > dataSize )
            bytes = dataSize;

        /* Copy data */
        for( i=0; i<bytes; i++ )
        {
           OSA_EnterCritical(kCriticalDisableInt);          
           pData[i] = pSer->rxBuffer[pSer->rxOut++];
            if ( pSer->rxOut >= gSMRxBufSize_c )
            {
                pSer->rxOut = 0;
            }
           OSA_ExitCritical(kCriticalDisableInt);
        }
    }

    return bytes;
}

/*! *********************************************************************************
* \brief   This function will mark all finished TX queue entries as empty.
*          If a calback was provided, it will be run.
*
* \param[in] pSer pointer to the serial interface internal structure
*
********************************************************************************** */
static void Serial_TxQueueMaintenance(serial_t *pSer)
{
    uint32_t i;

    while( pSer->txQueue[pSer->txOut].dataSize == 0 )
    {
        i = pSer->txOut;
        mSerial_IncIdx_d(pSer->txOut, gSerialMgrTxQueueSize_c);
        
        /* Run Calback */
        if( pSer->txQueue[i].txCallback )
        {
            pSer->txQueue[i].txCallback( pSer->txQueue[i].pTxParam );
            pSer->txQueue[i].txCallback = NULL;
        }

#if gSerialMgr_BlockSenderOnQueueFull_c && gSMGR_UseOsSemForSynchronization_c
        OSA_EnterCritical(kCriticalDisableInt);        
        if( pSer->txBlockedTasks )
        {
            pSer->txBlockedTasks--;
            OSA_ExitCritical(kCriticalDisableInt);
            (void)OSA_SemaPost(&pSer->txQueueSem);
        }
        else
        {
          OSA_ExitCritical(kCriticalDisableInt);
        }
#endif
        if( pSer->txOut == pSer->txIn )
            break;
    }
}

/*! *********************************************************************************
* \brief   This function will unblock the task who called Serial_SyncWrite().
*
* \param[in] pSer pointer to the serial interface internal structure
*
********************************************************************************** */
static void Serial_SyncTxCallback(void *pSer)
{
#if gSMGR_UseOsSemForSynchronization_c
    (void)OSA_SemaPost( &((serial_t *)pSer)->txSyncSem );
#endif
}

/*! *********************************************************************************
* \brief   This function will return the interfaceId for the specified interface
*
* \param[in] type     the interface type
* \param[in] channel  the instance of the interfacte
*
* \return The mSerials index for the specified interface type and channel
*
********************************************************************************** */
uint32_t Serial_GetInterfaceId(serialInterfaceType_t type, uint32_t channel)
{
    uint32_t i;
    
    for(i=0; i<gSerialManagerMaxInterfaces_c; i++)
    {
        if( (mSerials[i].serialType == type) && 
            (mSerials[i].serialChannel == channel) )
            return i;
    }
    
    return gSerialMgrInvalidIdx_c;
}

#if (gSerialMgrUseUart_c)
static void UartTxCb(uint32_t instance)
{
    instance = Serial_GetInterfaceId(gSerialMgrUart_c, instance);
    SerialManager_TxNotify(instance);
}
static void UartRxCb(uint32_t instance)
{
    instance = Serial_GetInterfaceId(gSerialMgrUart_c, instance);
    SerialManager_RxNotify(instance);
}
#endif

#if (gSerialMgrUseSPI_c)
static void SpiSlaveTxCb(uint32_t instance)
{
    instance = Serial_GetInterfaceId(gSerialMgrSPISlave_c, instance);
    SerialManager_TxNotify(instance);
}

static void SpiSlaveRxCb(uint32_t instance)
{
    instance = Serial_GetInterfaceId(gSerialMgrSPISlave_c, instance);
    SerialManager_RxNotify(instance);
}
#endif


#endif //#if (gSerialManagerMaxInterfaces_c)
