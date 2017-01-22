/**HEADER********************************************************************
* 
* Copyright (c) 2008, 2013 - 2014 Freescale Semiconductor;
* All Rights Reserved
*
* Copyright (c) 1989-2008 ARC International;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: virtual_com.c$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief  The file emulates a USB PORT as RS232 PORT.
*****************************************************************************/ 

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"
#include "virtual_com.h"
#include "SerialManager.h"
#include "VirtualComInterface.h"
#include "usb_cdc_pstn.h"
#if USBCFG_DEV_COMPOSITE
#error This application requires USBCFG_DEV_COMPOSITE defined zero in usb_device_config.h. Please recompile usbd with this option.
#endif

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include "fsl_device_registers.h"
#include <stdio.h>
#include <stdlib.h>
#endif


/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/
 
/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/
extern void SerialManager_TxNotify( void *pData );
extern void SerialManager_VirtualComRxNotify(uint8_t* pData, uint16_t dataSize, uint8_t interface);
/****************************************************************************
 * Global Variables
 ****************************************************************************/
extern usb_desc_request_notify_struct_t  desc_callback;

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/
 
/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Callback(uint8_t event_type, void* val,void* arg);
uint8_t USB_Notif_Callback(uint8_t event, uint16_t value, uint8_t ** data, uint32_t* size, void* arg); 
static void ReceiveArmingInCriticalSection(cdc_handle_t handle);
/*****************************************************************************
 * Local Variables 
 *****************************************************************************/
uint8_t g_line_coding[LINE_CODING_SIZE] = 
{
 /*e.g. 0x00,0x10,0x0E,0x00 : 0x000E1000 is 921600 bits per second */
	(LINE_CODE_DTERATE_IFACE>> 0) & 0x000000FF,
	(LINE_CODE_DTERATE_IFACE>> 8) & 0x000000FF,
	(LINE_CODE_DTERATE_IFACE>>16) & 0x000000FF, 		 
	(LINE_CODE_DTERATE_IFACE>>24) & 0x000000FF,
	 LINE_CODE_CHARFORMAT_IFACE,
	 LINE_CODE_PARITYTYPE_IFACE,
	 LINE_CODE_DATABITS_IFACE
};

uint8_t g_abstract_state[COMM_FEATURE_DATA_SIZE] = 
{
	 (STATUS_ABSTRACT_STATE_IFACE>>0) & 0x00FF,
	 (STATUS_ABSTRACT_STATE_IFACE>>8) & 0x00FF																		
};

uint8_t g_country_code[COMM_FEATURE_DATA_SIZE] = 
{
 (COUNTRY_SETTING_IFACE>>0) & 0x00FF,
 (COUNTRY_SETTING_IFACE>>8) & 0x00FF															  
};
static cdc_handle_t   mCdcHandle;
static uint8_t mSerialIndex;
static bool_t mReceiveArmed = FALSE;
static bool_t mDeviceConfigured = FALSE;
#if gVirtualCOMPort_DiscardTxOnCOMClose_d
static bool mComOpen = FALSE;
#endif
static uint8_t g_curr_recv_buf[DATA_BUFF_SIZE];

/*****************************************************************************
 * Local Functions
 *****************************************************************************/
 
/**************************************************************************//*!
 *
 * @name  USB_Get_Line_Coding
 *
 * @brief The function returns the Line Coding/Configuraion
 *
 * @param handle:        handle     
 * @param interface:     interface number     
 * @param coding_data:   output line coding data     
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Line_Coding(uint32_t handle, 
                                uint8_t interface, 
                                uint8_t * *coding_data)
{   
    UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *coding_data = g_line_coding;
        return USB_OK;  
    }
    
    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Line_Coding
 *
 * @brief The function sets the Line Coding/Configuraion
 *
 * @param handle: handle     
 * @param interface:     interface number     
 * @param coding_data:   output line coding data     
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Line_Coding(uint32_t handle, 
                                uint8_t interface, 
                                uint8_t * *coding_data)
{   
    uint8_t count;

    UNUSED_ARGUMENT(handle)
    
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* set line coding data*/
        for (count = 0; count < LINE_CODING_SIZE; count++) 
        {          
            g_line_coding[count] = *((*coding_data+USB_SETUP_PKT_SIZE) + count);
        }
        return USB_OK;  
    }
    
    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle
 * @param interface:     interface number     
 * @param feature_data:   output comm feature data     
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Abstract_State(uint32_t handle, 
                                uint8_t interface, 
                                uint8_t * *feature_data)
{   
    UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *feature_data = g_abstract_state;
        return USB_OK;  
    }
    
    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Get_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle:        handle     
 * @param interface:     interface number     
 * @param feature_data:   output comm feature data     
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Get_Country_Setting(uint32_t handle, 
                                    uint8_t interface, 
                                    uint8_t * *feature_data)
{   
    UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* get line coding data*/
        *feature_data = g_country_code;
        return USB_OK;  
    }
    
    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Abstract_State
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (ABSTRACT_STATE)
 * @param handle:        handle     
 * @param interface:     interface number     
 * @param feature_data:   output comm feature data     
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Abstract_State(uint32_t handle, 
                                uint8_t interface, 
                                uint8_t * *feature_data)
{   
    uint8_t count;
    UNUSED_ARGUMENT(handle)
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        /* set Abstract State Feature*/
        for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++) 
        {          
            g_abstract_state[count] = *(*feature_data + count);
        }
        return USB_OK; 
    }
    
    return USBERR_INVALID_REQ_TYPE;
}

/**************************************************************************//*!
 *
 * @name  USB_Set_Country_Setting
 *
 * @brief The function gets the current setting for communication feature
 *                                                  (COUNTRY_CODE)
 * @param handle: handle     
 * @param interface:     interface number     
 * @param feature_data:   output comm feature data     
 *
 * @return USB_OK                              When Successfull
 *         USBERR_INVALID_REQ_TYPE             when Error
 *****************************************************************************/
uint8_t USB_Set_Country_Setting(uint32_t handle, 
                                    uint8_t interface, 
                                    uint8_t * *feature_data)
{   
    uint8_t count;
    UNUSED_ARGUMENT (handle)
    
    /* if interface valid */
    if(interface < USB_MAX_SUPPORTED_INTERFACES)
    {
        for (count = 0; count < COMM_FEATURE_DATA_SIZE; count++) 
        {          
            g_country_code[count] = *(*feature_data + count);
        }
        return USB_OK; 
    }
    
    return USBERR_INVALID_REQ_TYPE;
}
/*****************************************************************************
*  
*	@name		 VirtualCom_Init
* 
*	@brief		 This function starts cdc device.
* 
*	@param		 None
* 
*	@return 	 None
**				  
*****************************************************************************/
void* VirtualCom_Init(uint8_t param)
{
    mSerialIndex = param;
    cdc_config_struct_t cdc_config;
    cdc_config.cdc_application_callback.callback = USB_App_Callback;
    cdc_config.cdc_application_callback.arg = &mCdcHandle;
    cdc_config.vendor_req_callback.callback = NULL;
    cdc_config.vendor_req_callback.arg = NULL;
    cdc_config.class_specific_callback.callback = USB_Notif_Callback;
    cdc_config.class_specific_callback.arg = &mCdcHandle;
    cdc_config.desc_callback_ptr =  &desc_callback;
    /* Always happend in control endpoint hence hard coded in Class layer*/
    
    /* Initialize the USB interface */
    USB_Class_CDC_Init(CONTROLLER_ID, &cdc_config, &mCdcHandle);
    return (void*)mCdcHandle;
}
/*****************************************************************************
*  
*	@name		 VirtualCom_Write
* 
*	@brief		 This function arms a send operation on cdc bulk in endpoint  
* 
*	@param		 None
* 
*	@return 	 None
**				  
*****************************************************************************/
serialStatus_t VirtualCom_Write(void* interface, uint8_t* pData, uint16_t dataSize)
{
  serialStatus_t status = gSerial_InternalError_c;
  usb_status usbStatus;
#if gVirtualCOMPort_DiscardTxOnCOMClose_d
  if(mComOpen)
#else
    if(mDeviceConfigured)    
#endif    
    {
      usbStatus = USB_Class_CDC_Send_Data((cdc_handle_t)interface, DIC_BULK_IN_ENDPOINT, pData, dataSize);
      if(usbStatus == USB_OK)
      {
        status = gSerial_Success_c;
      }
      
    }
  return status;
}
/*****************************************************************************
*  
*	@name		 VirtualCom_SMReadNotify
* 
*	@brief		 This function is called from SerialRead function in serial manager. It arms a new 
*                        receive on the cdc bulk out endpoint if it isn't already armed and if there is room in 
*                        the receive buffer for bulk out endpoint size characters       
*	@param		 
* 
*	@return 	 None
**				  
*****************************************************************************/
void VirtualCom_SMReadNotify(void* interface)
{
  if(mDeviceConfigured)
      {
        ReceiveArmingInCriticalSection((cdc_handle_t)interface);
      }
}
/*****************************************************************************
*  
*	@name		 ReceiveArmingInCriticalSection
* 
*	@brief		 manage to arm securely a receive on cdc bulk out endpoint
* 
*	@param		 None
* 
*	@return 	 None
**				  
*****************************************************************************/
static void ReceiveArmingInCriticalSection(cdc_handle_t handle)
{
  uint16_t bytesCount;
  uint16_t bytesFree;
  bool_t rx = FALSE;
  
  OSA_EnterCritical(kCriticalDisableInt);
  if(!mReceiveArmed)
  {
    Serial_RxBufferByteCount( mSerialIndex, &bytesCount );
    bytesFree = gSerialMgrRxBufSize_c - bytesCount;  
    if(bytesFree >= DIC_BULK_OUT_ENDP_PACKET_SIZE)
    {
      rx = TRUE;
      mReceiveArmed = TRUE;
    }  
  }
  OSA_ExitCritical(kCriticalDisableInt);
  if(rx)
  {
    USB_Class_CDC_Recv_Data(handle, DIC_BULK_OUT_ENDPOINT, g_curr_recv_buf, DIC_BULK_OUT_ENDP_PACKET_SIZE);
  }
}                          

/******************************************************************************
 * 
 *    @name        USB_App_Callback
 *    
 *    @brief       This function handles the callback  
 *                  
 *    @param       handle : handle to Identify the controller
 *    @param       event_type : value of the event
 *    @param       val : gives the configuration value 
 * 
 *    @return      None
 *
 *****************************************************************************/
void USB_App_Callback(uint8_t event_type, void* val,void* arg) 
{
    uint32_t handle;
    handle = *((uint32_t *)arg);
    if(event_type == USB_DEV_EVENT_BUS_RESET)
    {
        
        mDeviceConfigured = FALSE;
#if gVirtualCOMPort_DiscardTxOnCOMClose_d        
        mComOpen = FALSE;
#endif        
        mReceiveArmed = FALSE;
    }
    else if(event_type == USB_DEV_EVENT_CONFIG_CHANGED)
    {
        /* Schedule buffer for receive */
      mReceiveArmed = FALSE;
#if gVirtualCOMPort_DiscardTxOnCOMClose_d      
      mComOpen = FALSE;
#endif      
      mDeviceConfigured = *((uint16_t*)val);
      if(mDeviceConfigured)
      {
        ReceiveArmingInCriticalSection(handle);
      }
    }
    else if(event_type == USB_DEV_EVENT_ERROR)
    {
        /* add user code for error handling */
    }
    else if(event_type == USB_DEV_EVENT_SEND_COMPLETE)
    {
        
    }
    
    return;
}

/******************************************************************************
 * 
 *    @name        USB_Notif_Callback
 *    
 *    @brief       This function handles the callback for Get/Set report req  
 *                  
 *    @param       request  :  request type
 *    @param       value    :  give report type and id
 *    @param       data     :  pointer to the data 
 *    @param       size     :  size of the transfer
 *
 *    @return      status
 *                  USB_OK  :  if successful
 *                  else return error
 *
 *****************************************************************************/
 
uint8_t USB_Notif_Callback
(
uint8_t event, 
uint16_t value, 
uint8_t ** data, 
uint32_t* size,
void* arg
) 
{
  cdc_handle_t handle;
  uint8_t error = USB_OK;
  handle = *((cdc_handle_t *)arg);
  switch(event)
  {
  case GET_LINE_CODING:
    error = USB_Get_Line_Coding(handle, value, data);
    break;
  case GET_ABSTRACT_STATE:
    error = USB_Get_Abstract_State(handle, value, data);
    break;
  case GET_COUNTRY_SETTING:
    error = USB_Get_Country_Setting(handle, value, data);
    break;
  case SET_LINE_CODING:
    error = USB_Set_Line_Coding(handle, value, data);
    break;
  case SET_ABSTRACT_STATE:
    error = USB_Set_Abstract_State(handle, value, data);
    break;
  case SET_COUNTRY_SETTING:
    error = USB_Set_Country_Setting(handle, value, data);
    break;
  case USB_APP_CDC_CARRIER_ACTIVATED:
  case USB_APP_CDC_CARRIER_DEACTIVATED:  
  case USB_APP_CDC_DTE_ACTIVATED:
  case USB_APP_CDC_DTE_DEACTIVATED:  
#if gVirtualCOMPort_DiscardTxOnCOMClose_d                  
    {
      cdc_pstn_struct_t * cdc_pstn_ptr;
      
      if(mDeviceConfigured)
      {
        cdc_pstn_ptr = (cdc_pstn_struct_t *)((cdc_device_struct_t *)handle)->pstn_obj_ptr;
        if(cdc_pstn_ptr->dte_status  == gVirtualCOMPort_LineStateCOMOpen_d)
        {
          mComOpen = TRUE; 
        }
        else
        {
          mComOpen = FALSE; 
        }
        
      }
    }
    
#endif
    break;
  case USB_DEV_EVENT_DATA_RECEIVED:
    {
      SerialManager_VirtualComRxNotify(*data, (uint16_t)*size, mSerialIndex);             
      mReceiveArmed = FALSE;
      ReceiveArmingInCriticalSection(handle);
    }
    break;
  case USB_DEV_EVENT_SEND_COMPLETE:
    {
      if(size!= NULL)
      {
#if gVirtualCOMPort_EndTxWithEmptyPacket_d
        if ((*size != 0) && ((*size % DIC_BULK_IN_ENDP_PACKET_SIZE) == 0))
        {
          USB_Class_CDC_Send_Data(handle, DIC_BULK_IN_ENDPOINT, *data, 0);
        }                    
        else
        {
          SerialManager_TxNotify( (void*)mSerialIndex);
        }
        
#else                 
        
        SerialManager_TxNotify( (void*)mSerialIndex);
#endif                     
      }
      
    }
    break;
  default:
    error = USBERR_INVALID_REQ_TYPE;
  }
  
  return error;
}



    
    

/* EOF */
