/*!=================================================================================================
\file       enet_driver.c
\brief      This is a private source file for the Ethernet driver adapter. 
            
\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any 
\copyright  form - including copied, transcribed, printed or by any electronic means - without 
\copyright  specific written permission from Freescale.
\copyright  (c) Copyright 2014, Freescale, Inc.  All rights reserved.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "PE_Types.h"
#include "psptypes.h"

#include "enet_driver.h"
#include "fsl_enet_driver.h"
#include "fsl_phy_driver.h"

#include <string.h>
#include "embeddedtypes.h"
#include "memmanager.h"
#include "network_utils.h"
#include "FunctionLib.h"

/*==================================================================================================
Private macros
==================================================================================================*/

#define ENET_DEVICE_NB 0

#define DEVICE_MAC_ADDRESS 0x00, 0x04, 0x9F, 0x00, 0xFA, 0x5D 

/*==================================================================================================
Private type definitions
==================================================================================================*/
typedef void (_CODE_PTR_ INT_ISR_FPTR)(pointer);

/*! @brief Define ECB structure contains protocol type and it's related service function*/
typedef struct enetServiceStruct_tag
{
    uint16_t  protocol;
    void (* service)(void*,uint8_t *, uint32_t);
    void  *privateData;
    struct enetServiceStruct_tag *next;
}enetServiceStruct_t;

/*==================================================================================================
Private prototypes
==================================================================================================*/
uint32_t ENET_receive(void *enetHandle, enet_mac_packet_buffer_t *packetBuffer);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

static const uint8_t defaultEnetAddr[] = {DEVICE_MAC_ADDRESS};

static enet_dev_if_t enetDevIf;
static enet_mac_config_t enetMacCfg = 
{
    kEnetMaxFrameSize,  /*!< ENET receive buffer size*/
    ENET_RX_LARGE_BUFFER_NUM,  /*!< ENET receive large buffer number*/
    ENET_RX_BUFFER_ALIGNMENT,  /*!< ENET receive buffer alignment*/
    ENET_TX_BUFFER_ALIGNMENT,  /*!< ENET transmit buffer alignment*/
    ENET_RX_RING_LEN,        /*!< ENET receive bd number*/
    ENET_TX_RING_LEN,        /*!< ENET transmit bd number*/
    ENET_BD_ALIGNMENT,         /*!< ENET bd alignment*/
    {0},                /*!< ENET mac address*/
    kEnetCfgRmii,       /*!< ENET rmii interface*/
    kEnetCfgSpeed100M,  /*!< ENET rmii 100M*/    
    kEnetCfgFullDuplex, /*!< ENET rmii Full- duplex*/        
    false,              /*!< ENET no loop*/
    false,              /*!< ENET enable promiscuous*/
    false,              /*!< ENET txaccelerator enabled*/
    false,              /*!< ENET rxaccelerator enabled*/
    false,              /*!< ENET store and forward*/
    {true, true, false, false, false},  /*!< ENET rxaccelerator config*/
    {true, true, false},          /*!< ENET txaccelerator config*/
    false,              /*!< vlan frame support*/
    true,               /*!< PHY auto discover*/
    ENET_MII_CLOCK,     /*!< ENET MDC clock*/
#if FSL_FEATURE_ENET_SUPPORT_PTP
    ENET_PTP_RING_BUFFER_NUM,   /*!< ptp ring buffer number */
    false,
#endif
};

static enet_phy_config_t enetPhyCfg =
{34, false };

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern INT_ISR_FPTR _int_install_isr(_mqx_uint,INT_ISR_FPTR, pointer);

extern LDD_TDeviceData* EthResetPin_Init(LDD_TUserData *UserDataPtr);
extern void EthResetPin_ClrVal(LDD_TDeviceData *DeviceDataPtr);
extern void EthResetPin_SetVal(LDD_TDeviceData *DeviceDataPtr);

extern void ENET_Transmit_IRQHandler(void);
extern void ENET_Receive_IRQHandler(void);

/*==================================================================================================
Private functions
==================================================================================================*/

/*!*************************************************************************************************
\fn uint32_t ENET_receive(void *enetHandle, enet_mac_packet_buffer_t *packetBuffer)
\brief  Enet receive callback function.

\param [in]  enetHandle   handle to Ethernet driver
\param [in]  packetBuffer structure containing pointer to data and lenght
      
\retval      none
***************************************************************************************************/
uint32_t ENET_receive
(
    void *enetHandle, 
    enet_mac_packet_buffer_t *packetBuffer
)
{   
    uint32_t returnStatus = ENET_OK;
    enetServiceStruct_t* serviceStructPtr;
    //uint32_t index = 0;
    //uint32_t frameSize;
    uint16_t *typePtr;
    uint16_t type;
    enet_dev_if_t *enetDevifPtr = (enet_dev_if_t *)enetHandle;

    /* Process the received frame*/
    typePtr = &((enet_ethernet_header_t *)packetBuffer[0].data)->type;
    type = ntohs((*typePtr));
    if(type == ENETPROT_8021Q) 
    {
        typePtr = &((enet_8021vlan_header_t *)packetBuffer[0].data)->type;
        type = ntohs((*typePtr));
    }
    if(type <= kEnetMaxFrameDateSize)
    {
        enet_8022_header_ptr llcPtr = (enet_8022_header_ptr)(typePtr + 2);
        type = htons(llcPtr->type);
    }


    for(serviceStructPtr = (enetServiceStruct_t*)enetDevifPtr->netIfPtr; serviceStructPtr;
        serviceStructPtr = serviceStructPtr->next)
    {
        if(serviceStructPtr->protocol == type)
        {
#if 0
            while(0 != packetBuffer[index].length)
            {
                frameSize += packetBuffer[index].length;
                index++;
            }

            uint8_t* packet = MEM_BufferAlloc(frameSize);

            index = 0;
            frameSize = 0;

            while(0 != packetBuffer[index].length)
            {
                FLib_MemCpy(packet+frameSize,packetBuffer[index].data,packetBuffer[index].length);
                frameSize += packetBuffer[index].length;
                index++;
            }
#endif            
            serviceStructPtr->service(serviceStructPtr->privateData,packetBuffer[0].data,packetBuffer[0].length);           
        }
    }
  
    return returnStatus;
}

/*==================================================================================================
Public functions
==================================================================================================*/

void ENET_TxIsr
(
    void* param
)
{
    ENET_Transmit_IRQHandler();
}

void ENET_RxIsr
(
    void* param
)
{
    ENET_Receive_IRQHandler();
}


/*!*************************************************************************************************
\fn uint32_t ENET_get_address(void* enetHandle, uint8_t* address)
\brief  Retrieves the Ethernet address of a device.

\param [in]   enetHandle    handle to Ethernet driver        
\param [out]  address      mac address
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t ENET_get_address
(
      void* enetHandle,
      uint8_t* address
)
{   
    uint32_t returnStatus = ENET_OK;   

    /* Check input param*/
    if (NULL != enetHandle)
    {
        enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)enetHandle;
        FLib_MemCpy(address, enetIfPtr->macCfgPtr->macAddr, kEnetMacAddrLen);
    }
    else
    {
        returnStatus = ENETERR_INVALID_INIT_PARAM;
    }
    return returnStatus;

} 


/*!*************************************************************************************************
\fn uint32_t ENET_initialize(uint8_t* address,void* enetHandle)
\brief  Initializes the chip.
        
\param [in]  address       the local Ethernet address
\param [out] enetHandle    handle to Ethernet driver
       
\retval      uint32_t     ENET_OK
                          error code 
***************************************************************************************************/
uint32_t ENET_initialize
(
      uint8_t* address,
      void** enetHandle
)
{
    enet_dev_if_t * enetIfPtr;
    uint32_t result;
    uint32_t counter;
    uint32_t returnStatus = ENET_OK;
    LDD_TDeviceData* gpioPtr;
        
    /* init PHY reset Gpio */
    gpioPtr = EthResetPin_Init(NULL);

    EthResetPin_ClrVal(gpioPtr);
    for(counter = 0; counter <20000; counter++);
    EthResetPin_SetVal(gpioPtr);
    OSA_EXT_TimeDelay(2000);

    _int_install_isr(LDD_ivIndex_INT_ENET_Transmit,ENET_TxIsr, NULL);
    _int_install_isr(LDD_ivIndex_INT_ENET_Receive,ENET_RxIsr, NULL);
    
    /* Check the device status*/
    if (FALSE == enetDevIf.isInitialized)
    {
       /* Initialize device*/
        enetIfPtr = (enet_dev_if_t *)&enetDevIf;
        enetIfPtr->next = NULL;
        
        enetIfPtr->deviceNumber = ENET_DEVICE_NB;
        enetIfPtr->macCfgPtr = &enetMacCfg;
        enetIfPtr->phyCfgPtr = &enetPhyCfg;
        enetIfPtr->macApiPtr = &g_enetMacApi;
        enetIfPtr->phyApiPtr = (void *)&g_enetPhyApi;
        enetIfPtr->enetNetifcall = ENET_receive;

        if(NULL != address)
        {
            FLib_MemCpy(enetIfPtr->macCfgPtr->macAddr, address, kEnetMacAddrLen);
        }
        else
        {
            FLib_MemCpy(enetIfPtr->macCfgPtr->macAddr, (uint8_t*)defaultEnetAddr, kEnetMacAddrLen);
        }

        /* Create sync signal*/
        //lock_create(&enetIfPtr->enetContextSync);
        
        /* Initialize ENET device*/
        result = enetIfPtr->macApiPtr->enet_mac_init(enetIfPtr);
        if (result == kStatus_ENET_Success)
        {
            *enetHandle = enetIfPtr;
            enetIfPtr->isInitialized = true;
        }
        else
        {
            //lock_destroy(&enetIfPtr->enetContextSync);
            *enetHandle = NULL;
            returnStatus = ENET_ERROR;
        }
    }
    else
    {
        returnStatus = ENETERR_INITIALIZED_DEVICE;
    }

    return returnStatus;
}

/*!*************************************************************************************************
\fn uint32_t ENET_open(void* enetHandle, int16_t protocol, void (*service)(uint8_t *, uint32_t),
                       void* privateData)
\brief  Registers a protocol type on an Ethernet channel.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  protocol      the protocol to open
\param [in]  service       the callback function
\param [in]  privateData   private data for service function
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t ENET_open
(
    void* enetHandle,
    uint16_t protocol,
    void (*service)(void*, uint8_t *, uint32_t),
    void* privateData
      
)
{   
    enet_dev_if_t * enetIfPtr;
    enetServiceStruct_t* serviceStructPtr;
    enetServiceStruct_t** searchPtr;
    uint32_t returnStatus = ENET_OK;

    /* Check input parameter*/
    if (NULL != enetHandle)
    {
   
        enetIfPtr = (enet_dev_if_t *)enetHandle;
        //lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);

        for(searchPtr=(enetServiceStruct_t **)(&enetIfPtr->netIfPtr); *searchPtr; searchPtr=&(*searchPtr)->next)
        {
            if ((*searchPtr)->protocol == protocol)
            {
                //lock_release(&enetIfPtr->enetContextSync);
                returnStatus = ENETERR_OPEN_PROT;
                break;
            }
        }
        if(ENET_OK == returnStatus)
        {
            serviceStructPtr = (enetServiceStruct_t*)MEM_BufferAlloc(sizeof(enetServiceStruct_t));
            if (NULL != serviceStructPtr)
            {
                serviceStructPtr->protocol = protocol;
                serviceStructPtr->service = service;
                serviceStructPtr->privateData = privateData;
                serviceStructPtr->next = NULL;
                *searchPtr = serviceStructPtr;
                //lock_release(&enetIfPtr->enetContextSync);
            }
            else
            {
                //lock_release(&enetIfPtr->enetContextSync);
                returnStatus = ENETERR_ALLOC_ECB;
            }
        }
    }
    else
    {
        returnStatus = ENETERR_INVALID_DEVICE;
    }
    
    return returnStatus;
}

/*!*************************************************************************************************
\fn uint32_t ENET_close(void* enetHandle,uint16_t protocol)
\brief  Unregisters a protocol type on an Ethernet channel.

\param [in]  enetHandle   handle to Ethernet driver
\param [in]  protocol     the protocol to close
      
\retval      uint32_t     ENET_OK
                          error code 
***************************************************************************************************/
uint32_t ENET_close
(
    void* enetHandle,
    uint16_t protocol
)
{
    return ENET_OK;

}
/*!*************************************************************************************************
\fn uint32_t ENET_send(void* enetHandle,ipPktInfo_t* packet,uint16_t protocol, uint8_t* dest, 
                       uint32_t  flags)
\brief  Sends a packet.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  inData        the packet to send
\param [in]  protocol      the protocol to send
\param [in]  dest          the destination Ethernet address
\param [in]  flags         optional flags, zero = default
      
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t ENET_send
(   
    void* enetHandle,
    ipPktInfo_t* packet,
    uint16_t protocol,
    uint8_t*  dest,
    uint32_t  flags
)
{
    uint8_t headerLen;
    uint32_t size = 0, lenTemp = 0;
    uint32_t returnStatus = ENET_OK;

    nwkBuffer_t* pNwkBuffer = packet->pNwkBuff;
    enet_dev_if_t *enetIfPtr;
    enet_ethernet_header_t *packetPtr;
    uint8_t *frame;
    
    /*Check out*/
    if ((NULL != enetHandle) && (NULL != packet))
    {
        enetIfPtr = (enet_dev_if_t *)enetHandle;
        /* Default frame header size*/
        headerLen = sizeof(enet_ethernet_header_t);

        /* Check the frame length*/
        size = NWKU_NwkBufferTotalSize(packet->pNwkBuff);
        
        if (size < enetIfPtr->maxFrameSize)
        {
            /* Allocate for a frame */
            frame = enet_mac_dequeue_buffer((void * *)&enetIfPtr->macContextPtr->txBufferPtr);
            if (NULL != frame)
            {
                /*Add MAC hardware address*/
                packetPtr = (enet_ethernet_header_t *)frame;
                FLib_MemCpy(packetPtr->destAddr, dest,kEnetMacAddrLen);
                FLib_MemCpy(packetPtr->sourceAddr, enetIfPtr->macCfgPtr->macAddr,kEnetMacAddrLen);
                packetPtr->type = htons(protocol);
                
                if (flags & ENET_OPT_8021QTAG)
                {
                    enet_8021vlan_header_t *vlanHeadPtr = (enet_8021vlan_header_t *)packetPtr;
                    vlanHeadPtr->tpidtag = htons(ENETPROT_8021Q);
                    vlanHeadPtr->othertag = htons((ENET_GETOPT_8021QPRIO(flags) << 13));
                    vlanHeadPtr->type = htons(protocol);
                    headerLen = sizeof(enet_8021vlan_header_t);  
                }

                if (flags & ENET_OPT_8023)
                {
                    enet_8022_header_ptr lcPtr = (enet_8022_header_ptr)(packetPtr->type + 2);
                    packetPtr->type = htons(size - headerLen);
                    lcPtr->dsap[0] = 0xAA;
                    lcPtr->ssap[0] = 0xAA;
                    lcPtr->command[0] = 0x03;
                    lcPtr->oui[0] = 0x00;
                    lcPtr->oui[1] = 0x00;
                    lcPtr->oui[2] = 0x00;
                    lcPtr->type = htons(protocol);
                    headerLen += sizeof(enet_8022_header_t);
                }

                /* add the size of the enet header to total size of the frame */
                size += headerLen;
                                
                /* Send a whole frame with a signal buffer*/
                lenTemp = headerLen;
                while(NULL != pNwkBuffer)
                {
                    FLib_MemCpy(frame + lenTemp,pNwkBuffer->pData,pNwkBuffer->size);
                    lenTemp += pNwkBuffer->size;
                    pNwkBuffer = pNwkBuffer->next;
                }
             
                enetIfPtr->macApiPtr->enet_mac_send(enetIfPtr, frame, size);
            }
            else
            {
                    returnStatus = ENETERR_ALLOC;
            }
        }
        else
        {
            returnStatus = ENETERR_SEND_LARGE;
        }
    }
    else
    {
        returnStatus =  ENETERR_INVALID_INIT_PARAM;
    }

    /* Free the PCB buffer*/
    NWKU_FreeIpPktInfo(&packet);
    
    return returnStatus;
}  


/*!*************************************************************************************************
\fn uint32_t ENET_get_MTU(void* enetHandle)
\brief  Get the maximum transmission unit.

\param [in]  enetHandle   handle to Ethernet driver
      
\retval      uint32_t     ENET MTU
***************************************************************************************************/
uint32_t ENET_get_MTU
(
    void* enetHandle
)
{
    uint32_t mtuSize = ENET_OK;   

    /* Check input param*/
    if (NULL != enetHandle)
    {
        enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)enetHandle;
        if (!enetIfPtr->maxFrameSize)
        {
            mtuSize = kEnetMaxFrameDateSize;
        }

        if (enetIfPtr->macCfgPtr->isVlanEnabled)
        {
            mtuSize = enetIfPtr->maxFrameSize - sizeof(enet_ethernet_header_t) - kEnetFrameFcsLen;
        }
        else
        {
            mtuSize = enetIfPtr->maxFrameSize - sizeof(enet_8021vlan_header_t) - kEnetFrameFcsLen;
        } 
    }
    
    return mtuSize;
}

/*!*************************************************************************************************
\fn uint32_t ENET_join(void* enetHandle,uint8_t* address,uint16_t protocol)
\brief  Joins a multicast group on an Ethernet channel.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code 
***************************************************************************************************/
uint32_t ENET_join
(
      void* enetHandle,
      uint8_t* address,
      uint16_t protocol      
)
{
    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)enetHandle;
    enet_multicast_group_t *enetMultiGroupPtr;

    uint32_t returnStatus = ENET_OK;   

    /* Make sure it's a multicast group*/
    if (address[0] & 1U)
    {
        //lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);

        if (!enetIfPtr->multiGroupPtr)
        {
            enetIfPtr->multiGroupPtr = MEM_BufferAlloc(sizeof(enet_multicast_group_t));
            if (NULL != enetIfPtr->multiGroupPtr)
            {
                FLib_MemCpy(enetIfPtr->multiGroupPtr->groupAdddr, address, kEnetMacAddrLen); 
                enetIfPtr->macApiPtr->enet_add_multicast_group(enetIfPtr->deviceNumber, 
                                                               enetIfPtr->multiGroupPtr, address);
                enetIfPtr->multiGroupPtr->next = NULL;
                enetIfPtr->multiGroupPtr->prv = NULL;
            }
            else
            {
                //lock_release(&enetIfPtr->enetContextSync);
                returnStatus = ENETERR_ALLOC;
            }
        }
        else
        {
            /* Check if we had added allready the multicast group */
            enetMultiGroupPtr = enetIfPtr->multiGroupPtr;
            while (enetMultiGroupPtr != NULL)
            {
                if (FLib_MemCmp(enetMultiGroupPtr->groupAdddr, address, kEnetMacAddrLen))
                {
                    //lock_release(&enetIfPtr->enetContextSync);
                    returnStatus = ENETERR_INITIALIZED_MULTICAST;
                    break;
                }
                if (enetMultiGroupPtr->next == NULL)
                {
                    break;
                }
                enetMultiGroupPtr =  enetMultiGroupPtr->next;
            }

            if(ENET_OK == returnStatus)
            {
                /* Add this multicast group*/
                enetMultiGroupPtr->next = MEM_BufferAlloc(sizeof(enet_multicast_group_t));
               
                FLib_MemCpy(enetMultiGroupPtr->next->groupAdddr, address, kEnetMacAddrLen);
                enetIfPtr->macApiPtr->enet_add_multicast_group(enetIfPtr->deviceNumber, 
                                                              enetMultiGroupPtr->next, address);
                enetMultiGroupPtr->next->next = NULL;
                enetMultiGroupPtr->next->prv = enetMultiGroupPtr;              
            }
        }
        //lock_release(&enetIfPtr->enetContextSync);    
    }
    else
    {
        returnStatus = ENETERR_JOIN_MULTICAST;
    }
    
    return returnStatus;
}

/*!*************************************************************************************************
\fn uint32_t ENET_leave(void* enetHandle, uint8_t* address, uint16_t protocol)
\brief  Leaves a multicast group on an Ethernet channel.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code 
***************************************************************************************************/
uint32_t ENET_leave
(   
      void* enetHandle,
      uint8_t* address,
      uint16_t protocol
)
{

    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)enetHandle;
    enet_multicast_group_t *enetMultiGroupPtr, *enetTempPtr;
    uint32_t returnStatus = ENET_OK;
    
    /* Make sure it's a multicast group*/
    if (address[0] & 1U)
    {
        //lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);
        if (enetIfPtr->multiGroupPtr)
        {
            /* Check if we had add the multicast group*/
            for (enetMultiGroupPtr = enetIfPtr->multiGroupPtr; enetMultiGroupPtr != NULL;
                 enetMultiGroupPtr = enetMultiGroupPtr->next )
            {
                if (!FLib_MemCmp(enetMultiGroupPtr->groupAdddr, address, kEnetMacAddrLen))
                {
                    enetIfPtr->macApiPtr->enet_leave_multicast_group(enetIfPtr->deviceNumber, 
                                                                     enetMultiGroupPtr, address);
                    FLib_MemSet(enetMultiGroupPtr->groupAdddr, 0, kEnetMacAddrLen);
                    enetTempPtr = enetMultiGroupPtr->prv;

                    if (enetTempPtr != NULL)
                    {
                        enetTempPtr->next = enetMultiGroupPtr->next;
                    } 
                    if (enetMultiGroupPtr->next != NULL)
                    {
                        enetMultiGroupPtr->next->prv = enetTempPtr;
                    } 
                    MEM_BufferFree((void *)enetMultiGroupPtr);
                    break;
                }
            }
            //lock_release(&enetIfPtr->enetContextSync);   
        }
        else
        {
            //lock_release(&enetIfPtr->enetContextSync);
            return ENETERR_NULL_MULTICAST;
        }             
    }
    else
    {
         returnStatus = ENETERR_JOIN_MULTICAST;
    }

    return returnStatus;

}

/*================================================================================================*/

/*==================================================================================================
Private debug functions
==================================================================================================*/
