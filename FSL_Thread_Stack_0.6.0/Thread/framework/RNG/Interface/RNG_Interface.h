/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
* \file RNG_interface.h
* RNG implementation header file for the ARM CORTEX-M4 processor
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

#ifndef _RNG_INTERFACE_H_
#define _RNG_INTERFACE_H_

#include "EmbeddedTypes.h"


/******************************************************************************
*******************************************************************************
* Public macros
*******************************************************************************
******************************************************************************/
#define gRNG_NoHWSupport_d    0
#define gRNG_RNGAHWSupport_d  1
#define gRNG_RNGBHWSupport_d  2
#define gRNG_TRNGHWSupport_d  3

#ifndef gRNG_HWSupport_d
#define gRNG_HWSupport_d gRNG_NoHWSupport_d
#endif

#define gRngSuccess_d       (0x00)
#define gRngInternalError_d (0x01)
#define gRngNullPointer_d   (0x80)
#define gRngMaxRequests_d   (100000)


/******************************************************************************
*******************************************************************************
* Public type definitions
*******************************************************************************
******************************************************************************/

/******************************************************************************
*******************************************************************************
* Public memory declarations
*******************************************************************************
******************************************************************************/

/******************************************************************************
*******************************************************************************
* Public function prototypes
*******************************************************************************
******************************************************************************/

/******************************************************************************
 * Name: RNG_Init()
 * Description: Initialize the RNG HW module
 * Parameter(s): -
 * Return: Status of the RNG module
 ******************************************************************************/
uint8_t RNG_Init(void);

/******************************************************************************
 * Name: RNG_GetRandomNo()
 * Description: Reads a 32-bit statistically random number from the RNG module or from 802.15.4 PHY
 * Parameter(s): [OUT] pRandomNo - pointer to location where the RN will be stored
 * Return: none
 ******************************************************************************/
void RNG_GetRandomNo(uint32_t* pRandomNo);

/******************************************************************************
 * Name: PRNG_SetSeed()
 * Description: Initialize seed for the PRNG algorithm.
 * Parameter(s):
 *      pSeed - can be set using the RNG_GetRandomNo() function
 * Return: None
 ******************************************************************************/
void RNG_SetPseudoRandomNoSeed(uint8_t* pSeed);

/******************************************************************************
 * Name: PRNG_GetRandomNo()
 * Description: Generates a NIST FIPS Publication 186-2-compliant 160 bit pseudo-random number
 * Parameter(s):
 *      pOut - pointer to the output buffer
 *      outBytes - the number of bytes to be copyed (1-20)
 *      pXSEED - optional user SEED. Should be NULL if not used.
 * Return: The number of bytes copied or -1 if reseed is needed
 ******************************************************************************/
int16_t RNG_GetPseudoRandomNo(uint8_t* pOut, uint8_t outBytes, uint8_t* pXSEED);

#endif /* _RNG_INTERFACE_H_ */