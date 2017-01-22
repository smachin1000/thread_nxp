/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
* \file RNG.c
* RNG implementation file for the ARM CORTEX-M4 processor
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


#include "RNG_Interface.h"
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "SecLib.h"
#include "FunctionLib.h"

#if (gSecLib_HWSupport_d == gSecLib_MMCAUSupport_d)
#include "mmcau_interface.h"
#endif

#ifndef gRNG_UsePhyRngForInitialSeed_d
#define gRNG_UsePhyRngForInitialSeed_d 0
#endif

#if (gRNG_HWSupport_d == gRNG_NoHWSupport_d)
  uint32_t mRandomNumber;

  #if gRNG_UsePhyRngForInitialSeed_d
  extern void PhyGetRandomNo(uint32_t *pRandomNo);
  #endif
#endif

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define mPRNG_NoOfBits_c      (160)
#define mPRNG_NoOfBytes_c     (mPRNG_NoOfBits_c/8)
#define mPRNG_NoOfLongWords_c (mPRNG_NoOfBits_c/32)

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static uint32_t XKEY[mPRNG_NoOfLongWords_c];
static uint32_t mPRNG_Requests = gRngMaxRequests_d;

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/******************************************************************************
 * Name: RNG_Init()
 * Description: Initialize the RNG HW module
 * Parameter(s): -
 * Return: Status of the RNG module
 ******************************************************************************/
uint8_t RNG_Init(void)
{
#if gRNG_HWSupport_d == gRNG_RNGAHWSupport_d
    CLOCK_SYS_EnableRngaClock(0);
    /* Mask Interrupts and start RNG */
    RNG_CR = RNG_CR_INTM_MASK | RNG_CR_HA_MASK | RNG_CR_GO_MASK;

#elif gRNG_HWSupport_d == gRNG_RNGBHWSupport_d
    CLOCK_SYS_EnableRngbClock(0);
    /* Execute a SW Reset */
    RNG_CMD |= RNG_CMD_SR_MASK;

    /* Mask Interrupts */
    RNG_CR = RNG_CR_MASKDONE_MASK | RNG_CR_MASKERR_MASK;

    /* Start Self Test and Seed Generation */
    RNG_CMD = RNG_CMD_ST_MASK | RNG_CMD_GS_MASK;

    /* Wait for Self Test and Seed Generation to be done*/
    while (RNG_CMD & (RNG_CMD_ST_MASK | RNG_CMD_GS_MASK));

    /* Enable RNG Auto-Reseed */
    RNG_CR |= RNG_CR_AR_MASK;

    /* Check for Errors */
    if ( RNG_SR & RNG_SR_ERR_MASK )
    {
        return (uint8_t)(RNG_ESR);
    }
#elif gRNG_HWSupport_d == gRNG_TRNGHWSupport_d
//    uint32_t temp;
//
//    SIM_SCGC6 |= SIM_SCGC6_TRNG_MASK;
//
//    /* Reset TRNG registers to default values */
//    HW_TRNG_RTMCTL_WR(gTRNG_BaseAddr_c, BM_TRNG_RTMCTL_RST_DEF | BM_TRNG_RTMCTL_PRGM);
//
//    /* Enable Entropy Valid IRQ */
//    //disallow device to sleep, allow device to sleep in ISR 
//    //HW_TRNG_SA_TRNG_INT_MASK_SET(gTRNG_BaseAddr_c, BM_TRNG_SA_TRNG_INT_MASK_SA_TRNG_SBS_ENTROPY_VALID);
//
//    /* Set TRNG in Run mode, and enable entropy read access */
//    temp = HW_TRNG_RTMCTL_RD(gTRNG_BaseAddr_c);
//    temp &= ~(BM_TRNG_RTMCTL_PRGM | BM_TRNG_RTMCTL_ERR);
//    temp |= BM_TRNG_RTMCTL_TRNG_ACC;
//    HW_TRNG_RTMCTL_WR(gTRNG_BaseAddr_c, temp);
#else
    #if gRNG_UsePhyRngForInitialSeed_d
    PhyGetRandomNo(&mRandomNumber);
    #else
    mRandomNumber = HW_SIM_UIDL_RD(SIM_BASE);
    #endif
#endif /* gRNG_HwSupport_d == 1 */

    /* Init Successfull */
    return gRngSuccess_d;
}

/******************************************************************************
 * Name: RNG_HwGetRandomNo()
 * Description: Read a random number from the HW RNG module
 * Parameter(s): [OUT] pRandomNo - pointer to location where the RN will be stored
 * Return: status of the RNG module
 ******************************************************************************/
#if (gRNG_HWSupport_d != gRNG_NoHWSupport_d)
static uint8_t RNG_HwGetRandomNo(uint32_t* pRandomNo)
{
#if gRNG_HWSupport_d == gRNG_RNGAHWSupport_d
    /* If output register is empty, wait for a new random number */
    while ( ((RNG_SR & RNG_SR_OREG_LVL_MASK) >> RNG_SR_OREG_LVL_SHIFT) == 0 );

    /* Copy the output of RNG module */
    *pRandomNo = RNG_OR;

#elif gRNG_HWSupport_d == gRNG_RNGBHWSupport_d
    /* Check for Errors */
    if ( RNG_SR & RNG_SR_ERR_MASK )
        return (uint8_t)(RNG_ESR);

    /* If output FIFO is empty, wait for a new random number */
    while (((RNG_SR & RNG_SR_FIFO_LVL_MASK) >> RNG_SR_FIFO_LVL_SHIFT) == 0 );

    /* Copy the output of RNG module */
    *pRandomNo = RNG_OUT;

#elif gRNG_HWSupport_d == gRNG_TRNGHWSupport_d
    static uint8_t entropyIdx = 0;
    
    /* wait for entropy to be generated or for an error */
    while( !(HW_TRNG_RTMCTL_RD(gTRNG_BaseAddr_c) & (BM_TRNG_RTMCTL_ENT_VAL | BM_TRNG_RTMCTL_ERR)) );
    
    if( HW_TRNG_RTMCTL_RD(gTRNG_BaseAddr_c) & BM_TRNG_RTMCTL_ERR )
        return gRngInternalError_d;

    *pRandomNo = HW_TRNG_RTENTan_RD(gTRNG_BaseAddr_c, entropyIdx);
    if( ++entropyIdx == 16 )
    {
        entropyIdx = 0;
        //disallow device to sleep
    }
#endif

    return gRngSuccess_d;
}
#endif /* gRNG_HwSupport_d */


/******************************************************************************
 * Name: RNG_GetRandomNo()
 * Description: Read a random number from RNG module or from 802.15.4 PHY
 * Parameter(s): [OUT] pRandomNo - pointer to location where the RN will be stored
 * Return: none
 ******************************************************************************/
void RNG_GetRandomNo(uint32_t* pRandomNo)
{
    /* Check for NULL pointers */
    if (NULL == pRandomNo)
        return;

#if (gRNG_HWSupport_d == gRNG_NoHWSupport_d)
    mRandomNumber = (mRandomNumber * 6075) + 1283;
    FLib_MemCpy(pRandomNo, &mRandomNumber, sizeof(uint32_t));    
#else
    (void)RNG_HwGetRandomNo(pRandomNo);
#endif
}

/******************************************************************************
 * Name: RNG_SetPseudoRandomNoSeed()
 * Description: Initialize seed for the PRNG algorithm.
 * Parameter(s):
 *      pSeed - pointer to a buffer containing 20 bytes (160 bits).
 *             Can be set using the RNG_GetRandomNo() function.
 * Return: None
 ******************************************************************************/
void RNG_SetPseudoRandomNoSeed(uint8_t* pSeed)
{
    mPRNG_Requests = 1;
    FLib_MemCpy( XKEY, pSeed, mPRNG_NoOfBytes_c );
}

/******************************************************************************
 * Name: RNG_GetRandomNo()
 *
 * Description: Pseudo Random Number Generator (PRNG) implementation
 *              according to NIST FIPS Publication 186-2, APPENDIX 3
 *
 * Let x be the signer's private key.  The following may be used to generate m values of x:
 *   Step 1. Choose a new, secret value for the seed-key, XKEY.
 *   Step 2. In hexadecimal notation let
 *     t = 67452301 EFCDAB89 98BADCFE 10325476 C3D2E1F0.
 *     This is the initial value for H0 || H1 || H2 || H3 || H4 in the SHS.
 *   Step 3. For j = 0 to m - 1 do
 *     a. XSEEDj = optional user input.
 *     b. XVAL = (XKEY + XSEEDj) mod 2^b
 *     c. xj = G(t,XVAL) mod q
 *     d. XKEY = (1 + XKEY + xj) mod 2^b
 *
 * Parameter(s):
 *      pOut - pointer to the output buffer
 *      outBytes - the number of bytes to be copyed (1-20)
 *      pXSEED - optional user SEED. Should be NULL if not used.
 *
 * Return: The number of bytes copied or -1 if reseed is needed
 ******************************************************************************/
int16_t RNG_GetPseudoRandomNo(uint8_t* pOut, uint8_t outBytes, uint8_t* pXSEED)
{
    uint32_t i;
    sha1Context_t ctx;

    if (mPRNG_Requests == gRngMaxRequests_d)
        return -1;

    mPRNG_Requests++;

    /* a. XSEEDj = optional user input. */
    if (pXSEED)
    {
        /* b. XVAL = (XKEY + XSEEDj) mod 2^b */
        for (i=0; i<mPRNG_NoOfBytes_c; i++)
        {
            ctx.buffer[i] = ((uint8_t*)XKEY)[i] + pXSEED[i];
        }
    }
    else
    {
        for (i=0; i<mPRNG_NoOfBytes_c; i++)
        {
            ctx.buffer[i] = ((uint8_t*)XKEY)[i];
        }
    }

    /* c. xj = G(t,XVAL) mod q
    ***************************/
    SHA1_Hash(&ctx, ctx.buffer, mPRNG_NoOfBytes_c);

    /* d. XKEY = (1 + XKEY + xj) mod 2^b */
    XKEY[0] += 1;
    for (i=0; i<mPRNG_NoOfLongWords_c; i++)
    {
        XKEY[i] += ctx.hash[i];
    }

    /* Check if the length provided exceeds the output data size */
    if (outBytes > mPRNG_NoOfBytes_c)
    {
        outBytes = mPRNG_NoOfBytes_c;
    }

    /* Copy the generated number */
    for (i=0; i < outBytes; i++)
    {
        pOut[i] = ((uint8_t*)ctx.hash)[i];
    }

    return outBytes;
}

/********************************** EOF ***************************************/
