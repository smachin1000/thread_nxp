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

#ifndef _IP_IF_MANAGEMENT_H_
#define _IP_IF_MANAGEMENT_H_
/*!=================================================================================================
\file       ip_if_management.h
\brief      This is a header file for the IP interface management module.
==================================================================================================*/

#include "ip.h"
#include "network_utils.h"
#include "embeddedtypes.h"
/*==================================================================================================
Include Files
==================================================================================================*/

/*==================================================================================================
Public macros
==================================================================================================*/

/*==================================================================================================
Public type definitions
==================================================================================================*/

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
\fn ifHandle_t* IP_IF_Add(void*  driverHandle, mediaIfStruct_t * pIfStruct, uint16_t ipVersEnabled)
\brief  Adds a new interface to the global interface table.

\param [in]  driverHandle    the packet driver handle(can be NULL)
\param [in]  pIfStruct       call table for the interface
\param [in]  ipVersEnabled   the ip version that wants to be enabled on this interface(gIpProtv4_c,
                             gIpProtv6_c or gIpProtv4_c | gIpProtv6_c)

\retval      ifHandle_t*     pointer to array entry pointing to IP interface struct
***************************************************************************************************/
ifHandle_t* IP_IF_Add(void* driverHandle, mediaIfStruct_t * pIfStruct, uint16_t ipVersEnabled);

/*!*************************************************************************************************
\fn ifHandle_t* IP_IF_GetNullInterface(void)
\brief  Returns double pointer to NULL interface.

\retval      ifHandle_t*    returns pointer to NULL interface
***************************************************************************************************/
ifHandle_t* IP_IF_GetNullInterface(void);

/*!*************************************************************************************************
\fn int32_t IP_IF_GetIfNumber(ifHandle_t* pIfHandle)
\brief  Returns the index (from zero) in the interface table of the provided interface.

\param [in]  pIfHandle     double pointer to IP interface struct

\retval      int32_t       interface index or -1 in case of error
***************************************************************************************************/
int32_t IP_IF_GetIfNumber(ifHandle_t* pIfHandle);
/*!*************************************************************************************************
\fn bool_t IP_IF_IsMyAddr(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr)
\brief  Checks if an unicast address is attached/bound to the interface.

\param [in]  pIfHandle       double pointer to IP interface struct
\param [in]  pIpAddr         pointer to ip address

\retval      bool_t          TRUE if the address is attached/bound
                             FALSE otherwise
***************************************************************************************************/
bool_t IP_IF_IsMyAddr(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr);
/*!*************************************************************************************************
\fn void IP_IF_Join(ifHandle_t* ifPtr, ipAddr_t*groupIp)
\brief  Adds a multicast group into the physical interface

\param [in]  ifPtr           double pointer to IP interface struct
\param [in]  groupIp         the ip multicast address to join

\retval      none
***************************************************************************************************/
void IP_IF_Join(ifHandle_t* ifPtr, ipAddr_t*groupIp);
/*!*************************************************************************************************
\fn void IP_IF_Leave(ifHandle_t* ifPtr, ipAddr_t*groupIp)
\brief  Removes a multicast group from the physical interface

\param [in]  ifPtr           double pointer to IP interface struct
\param [in]  groupIp         the ip multicast group address to leave

\retval      none
***************************************************************************************************/
void IP_IF_Leave(ifHandle_t* ifPtr, ipAddr_t*groupIp);
/*!*************************************************************************************************
\fn ifHandle_t* IP_IF_GetIfByNr(uint32_t ifNumber)
\brief  Returns  double pointer to ifNumber interafce according to its index (from zero).

\param [in]  ifNumber       the interface number

\retval      ifHandle_t*    It returns NULL if there is no interface with the ifNumber index
***************************************************************************************************/
ifHandle_t* IP_IF_GetIfByNr(uint32_t ifNumber);
/*!*************************************************************************************************
\fn ifHandle_t* IP_IF_GetIfByAddr(ipAddr_t* pIpAddr)
\brief  This function returns double pointer to interafce which has the provided address.

\param [in]  pIpAddr         the ip address

\retval      ifHandle_t*     It returns NULL if there is no interface with the address
***************************************************************************************************/
ifHandle_t* IP_IF_GetIfByAddr(ipAddr_t* pIpAddr);

#if IP_IP4_ENABLE
/*!*************************************************************************************************
\fn ip4IfAddrData_t* IP_IF_GetAddressDataStruct4(ifHandle_t* pIfHandle, uint32_t ipAddr)
\brief  Gets the address data struct of the provided IPv4 address.

\param [in]  pIfHandle         double pointer to IP interface struct
\param [in]  ipAddr            the ip address

\retval      ip6IfAddrData_t*  poitner to address data struct or NULL if not found
***************************************************************************************************/
ip4IfAddrData_t* IP_IF_GetAddressDataStruct4(ifHandle_t* pIfHandle,uint32_t ipAddr);

/*!*************************************************************************************************
\fn uint32_t IP_IF_BindAddr4(ifHandle_t* pIfHandle, uint32_t ipAddr, uint32_t subnetMask,uint32_t defaultGw)
\brief  Binds an IP address to a hardware interface.

\param [in]  pIfHandle       double pointer to IP interface struct
\param [in]  ipAddr          the ip address to set. Must be in class A,B or C
\param [in]  subnetMask      subet mask of the ip address
\param [in]  defaultGw       default gateway for this source address

\retval      uint32_t        gIpOk_c
                             gIpInvalidParameterError_c
                             gIpNoAddressSpaceError_c
***************************************************************************************************/
uint32_t IP_IF_BindAddr4(ifHandle_t* pIfHandle, uint32_t ipAddr, uint32_t subnetMask,uint32_t defaultGw);
/*!*************************************************************************************************
\fn bool_t IP_IF_IsNotBound4(ifHandle_t* pIfHandle)
\brief Checks if an interface is not bound to any ip address.

\param [in]  pIfHandle      double pointer to IP interface struct

\retval      bool_t         TRUE or FALSE
***************************************************************************************************/
bool_t IP_IF_IsNotBound4(ifHandle_t* pIfHandle);
/*!*************************************************************************************************
\fn uint32_t IP_IF_UnbindAddr4(ifHandle_t* pIfHandle, uint32_t ipAddr)
\brief  Unbinds an IP address from a hardware interface.

\param [in]     pIfHandle       double pointer to IP interface struct
\param [in]     ipAddr          the ip address we want to unbind

\retval         gIpOk_c if ok
\retval         gIpInvalidParameterError_c if error
***************************************************************************************************/
uint32_t IP_IF_UnbindAddr4(ifHandle_t* pIfHandle, uint32_t ipAddr);

/*!*************************************************************************************************
\fn bool_t IP_IF_IsMyBcastAddr4(ifHandle_t* pIfHandle,ipAddr_t *pIpAddr)
\brief  Checks if a IPv4 broadcast address is attached/bound to the interface.

\param [in]  pIfHandle       double pointer to IP interface struct
\param [in]  pIpAddr         the ip address

\retval      bool_t          TRUE if the address is attached/bound
                             FALSE otherwise
***************************************************************************************************/
bool_t IP_IF_IsMyBcastAddr4(ifHandle_t* pIfHandle,ipAddr_t *pIpAddr);

/*!*************************************************************************************************
\fn uint32_t IP_IF_SelSrcAddr4(ipAddr_t* pDestAddr, ifHandle_t* pIfDest)
\brief  Selects the best IPv4 source address to use with an IPv4 destination address. Based on RFC
        1122

\param [in]  pDestAddr      the IPv4 address
\param [in]  pIfDest        double pointer to interface - used for broadcast and multicast

\retval      uint32_t       ip address ta struct or IP4_ADDR_ANY if not found
***************************************************************************************************/
uint32_t IP_IF_SelSrcAddr4(ipAddr_t* pDestAddr, ifHandle_t* pIfDest);

/*!*************************************************************************************************
\fn ip4IfAddrData_t* IP_IF_GetAddrByIf4(ifHandle_t* pIfHandle)
\brief  Searches the global IPv4 table for the addres correspoonding to the pIfHandle interface

\param [in]  pIfHandle          double pointer to IP interface struct

\retval      ip4IfAddrData_t    IPv4 address data struct of the interface
***************************************************************************************************/
ip4IfAddrData_t* IP_IF_GetAddrByIf4(ifHandle_t* pIfHandle);

#endif

#if IP_IP6_ENABLE
/*!*************************************************************************************************
\fn uint32_t IP_IF_AddMulticastGroup6(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr)
\brief  Adds a new multicast group for an interface.

\param [in]  pIfHandle      double pointer to IP interface struct
\param [in]  pIpAddr        the ip multicast address

\retval      uint32_t       gIpOk_c or error
***************************************************************************************************/
uint32_t IP_IF_AddMulticastGroup6(ifHandle_t* pIfHandle,  ipAddr_t* pIpAddr);
/*!*************************************************************************************************
\fn bool_t IP_IF_IsMyMulticastGroupAddr6(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr)
\brief  Checks if an address belongs to a multicast group regiesterd on the interface.

\param [in]  pIfHandle      double pointer to IP interface struct
\param [in]  pIpAddr        the ip multicast address

\retval      uint32_t       gIpOk_c or error
***************************************************************************************************/
bool_t IP_IF_IsMyMulticastGroupAddr6(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr);

/*!*************************************************************************************************
\fn bool_t IP_IF_HasPrefferedLLAddr6(ifHandle_t* pIfHandle)
\brief  Checks if an interface has a Preferred Link Local address.

\param [in]  pIfHandle      double pointer to IP interface struct

\retval      bool_t         true if link local address is preferred
***************************************************************************************************/
bool_t IP_IF_HasPrefferedLLAddr6(ifHandle_t* pIfHandle);

/*!*************************************************************************************************
\fn ip6IfAddrData_t** IP_IF_GetAddressDataStruct6(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr)
\brief  Gets the address data struct of the provided ipv6 address.

\param [in]  pIfHandle         double pointer to IP interface struct
\param [in]  pIpAddr           the ip address

\retval      ip6IfAddrData_t**  double pointer to address data struct or NULL if not found
***************************************************************************************************/
ip6IfAddrData_t** IP_IF_GetAddressDataStruct6(ifHandle_t* pIfHandle, ipAddr_t* pIpAddr);

/*!*************************************************************************************************
\fn bool_t IP_IF_IsMySolicitedMcastAddr6(ifHandle_t* pIfHandle,ipAddr_t *pIpAddr)
\brief  Checks if a solicited multicast address is attached/bound to the interface.

\param [in]  pIfHandle       double pointer to IP interface struct
\param [in]  pIpAddr         the ip address

\retval      bool_t          TRUE if the address is attached/bound
                             FALSE otherwise
***************************************************************************************************/
bool_t IP_IF_IsMySolicitedMcastAddr6(ifHandle_t* pIfHandle, ipAddr_t *pIpAddr);

/*!*************************************************************************************************
\fn ip6IfAddrData_t* IP_IF_GetAddrByIf6(ifHandle_t* pIfHandle, uint32_t addrIndex)
\brief  Searches the global IPv6 table for the addres with addrIndex index correspoonding to the
        pIfHandle interface

\param [in]  pIfHandle          double pointer to IP interface struct
\param [in]  addrIndex          index of address assigned to an interface

\retval      ip6IfAddrData_t*   pointer to address data struct
***************************************************************************************************/
ip6IfAddrData_t* IP_IF_GetAddrByIf6(ifHandle_t* pIfHandle, uint32_t addrIndex);
/*!*************************************************************************************************
\fn ip6IfAddrData_t** IP_IF_GetAddrByNr6(uint32_t addrNumber)
\brief  Returns a double pointer to addrNumber IPv6 address according to its index (from zero).

\param [in]  addrNumber     the address number

\retval      ifHandle_t     It returns NULL if there is no address with the addrNumber index
***************************************************************************************************/
ip6IfAddrData_t** IP_IF_GetAddrByNr6(uint32_t addrNumber);

/*!*************************************************************************************************
\fn void IP_IF_AssignScopeId6(ifHandle_t* iHandle)
\brief  This function assignes unique Scope ID to the interface.

\param [in]  iHandle        double pointer to IP interface struct

\retval      none
***************************************************************************************************/
void IP_IF_AssignScopeId6(ifHandle_t* iHandle);
/*!*************************************************************************************************
\fn ifHandle_t* IP_IF_GetIfByScopeId6(uint32_t scopeId)
\brief  This function returns pointer the interafce according its Scope ID.

\param [in]  scopeId         pointer to IP interface struct

\retval      ifHandle_t*      It returns NULL if there is no interface with the scope id
***************************************************************************************************/
ifHandle_t* IP_IF_GetIfByScopeId6(uint32_t scopeId);
/*!*************************************************************************************************
\fn ipAddr_t * IP_IF_SelSrcAddr6(ifHandle_t* pIfHandle, ipAddr_t* destAddr)
\brief  Selects the best source address to use with a destination address, based on RFC3484.

\param [in]  pIfHandle     double pointer to IP interface struct - Optional
\param [in]  destAddr      the ip address

\retval      ipAddr_t*     poitner to IPv6 address or NULL if not found
***************************************************************************************************/
ipAddr_t* IP_IF_SelSrcAddr6(ifHandle_t* pIfHandle, ipAddr_t* destAddr);

/*!*************************************************************************************************
\fn uint32_t IP_IF_BindAddr6(ifHandle_t* pIfHandle, ipAddr_t *ipAddr, ip6AddrType_t addrType,
             uint32_t lifetime, uint8_t prefixLength)
\brief  Binds an IP address to a hardware interface.

\param [in]     pIfHandle       double pointer to IP interface struct
\param [in,out] ipAddr          the ip address or prefix if we want to use autoconfiguration(NULL to
                                use link local prefix)
\param [in]     addrType        manual/autoconfigurable address
\param [in]     lifetime        address lifetime(IP6_ADDRESS_LIFETIME_INFINITE)
\param [in]     prefixLength    prefix length

\retval         uint32_t        gIpOk_c
                                gIpInvalidParameterError_c
                                gIpNoAddressSpaceError_c
***************************************************************************************************/
uint32_t IP_IF_BindAddr6(ifHandle_t* pIfHandle, ipAddr_t *ipAddr, ip6AddrType_t addrType,
                                  uint32_t lifetime, uint8_t prefixLenght);
/*!*************************************************************************************************
\fn uint32_t IP_IF_UnbindAddr6(ifHandle_t* pIfHandle, ipAddr_t *ipAddr)
\brief  Unbinds an IP address from a hardware interface.

\param [in]     pIfHandle         double pointer to IP interface struct
\param [in]     ipAddr          the ip address or prefix if we want to unbind

\retval         uint32_t        gIpOk_c
                                gIpInvalidParameterError_c
***************************************************************************************************/
uint32_t IP_IF_UnbindAddr6(ifHandle_t* pIfHandle, ipAddr_t *ipAddr);

#endif /* IP_IP6_ENABLE */

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*_IP_IF_MANAGEMENT_H_ */


