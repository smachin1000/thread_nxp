#ifndef _IP_IF_VTUN_H
#define _IP_IF_VTUN_H
/*!=================================================================================================
\file       ip_if_vtun.h
\brief      This is a header file for the Media interface to Virtual Enet. 

\copyright  Freescale Confidential Proprietary. No part of this document must be reproduced in any 
            form - including copied, transcribed, printed or by any electronic means - without 
            specific written permission from Freescale.
            (c) Copyright 2013, Freescale, Inc.  All rights reserved.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "ip.h"

/*==================================================================================================
Public macros
==================================================================================================*/


/*==================================================================================================
Public type definitions
==================================================================================================*/


/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern const mediaIfStruct_t* gVirtualTunMediaIfPtr;

/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn  void IP_vtunRecv(ipPktInfo_t* pIpPktInfo)

\brief  Sends a packet from VTUN interface to IP.

\param [in]  pIpPktInfo    the received packet
\param [in]  size          the size of the received packet

\retval      none
***************************************************************************************************/
void IP_vtunRecv(uint8_t* pInData, uint32_t size);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*_IP_IF_VTUN_H */
