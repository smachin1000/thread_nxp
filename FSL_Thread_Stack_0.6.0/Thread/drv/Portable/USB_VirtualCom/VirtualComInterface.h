/**HEADER********************************************************************
* 
* Copyright (c) 2008, 2013 Freescale Semiconductor;
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
* $FileName: virtual_com.h$
* $Version : 
* $Date    : 
*
* Comments:
*
* @brief The file contains Macro's and functions needed by the virtual com 
*        application
*
*****************************************************************************/

#ifndef _VIRTUAL_COM_INTERFACE_H
#define _VIRTUAL_COM_INTERFACE_H  1



/******************************************************************************
 * Constants - None
 *****************************************************************************/

/******************************************************************************
 * Macro's
 *****************************************************************************/
/* if gVirtualCOMPortSerialNoEnable_d == TRUE the Virtual Com device contains a Serial Number String Descriptor */

 #ifndef gVirtualCOMPortSerialNoEnable_d
  #define gVirtualCOMPortSerialNoEnable_d                       1
 #endif

#ifndef gDefaultValueOfVirtualCOMPortSerialNo_c
                                                            /*00000001*/
 #define gDefaultValueOfVirtualCOMPortSerialNo_c            0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x30,0x00,0x31,0x00
#endif   

#ifndef gVirtualCOMPort_DiscardTxOnCOMClose_d
  #define gVirtualCOMPort_DiscardTxOnCOMClose_d    0
#endif


#if gVirtualCOMPort_DiscardTxOnCOMClose_d   
  #define gVirtualCOMPort_LineStateCOMOpen_d     0x3
  #define gVirtualCOMPort_LineStateCOMClose_d    0x2
#endif

#ifndef gVirtualCOMPort_EndTxWithEmptyPacket_d      
  #define gVirtualCOMPort_EndTxWithEmptyPacket_d         1
#endif   
/*****************************************************************************
 * Global variables
 *****************************************************************************/

/*****************************************************************************
 * Global Functions
 *****************************************************************************/
extern void* VirtualCom_Init(uint8_t);
extern serialStatus_t VirtualCom_Write(void* interface, uint8_t* pData, uint16_t dataSize);
extern void VirtualCom_SMReadNotify(void* interface);

#endif 


/* EOF */
