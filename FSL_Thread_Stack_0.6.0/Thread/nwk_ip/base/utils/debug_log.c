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

/*!=================================================================================================
\file       debug_log.c
\brief      Use this module to print debug messages.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

#include "debug_log.h"
#if DEBUG_LOG
#include "EmbeddedTypes.h"
#include "FunctionLib.h"
#include "TimersManager.h"
#include "MemManager.h"
#include "panic.h"
#include "fsl_osa_ext.h"

/*==================================================================================================
Private macros
==================================================================================================*/

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
#ifdef MEM_TRACKING

#undef _block_size_
#undef _number_of_blocks_
#undef _eol_

#define _block_size_ 0 *
#define _number_of_blocks_ + 0 *
#define _eol_  + 1 +

uint16_t const poolCount = (PoolsDetails_c 0);

#undef _block_size_
#undef _number_of_blocks_
#undef _eol_

#define _block_size_ 0*
#define _number_of_blocks_ +
#define _eol_  +

uint16_t const mTotalNoOfMsgs_d  = (PoolsDetails_c 0);

#undef _block_size_
#undef _number_of_blocks_
#undef _eol_

extern const uint32_t heapSize;
extern uint8_t memHeap[];
extern blockTracking_t memTrack[];
extern pools_t memPools[];


#endif /* end MEM_TRACKING */


/*==================================================================================================
Public functions
==================================================================================================*/
void DBG_HexDump(uint8_t *pBuf, uint16_t buflen) {
  int n = 0;

  while (buflen--) {
    shell_printf("%02X ", *pBuf++);

    n++;
    if (n % 8 == 0) {
      if (n % 16 == 0)
	shell_printf("\n\r");
      else
	shell_printf(" ");
    }
  }
}


void DBG_PrintLog( bool_t printTimeStamp,  uint8_t* pString,
                    uint8_t* pBuf, size_t buflen)
{
    if(printTimeStamp)
    {
        DBG_PrintTimeStamp();
    }

	if(pString)
	{
		//print string message
	    DBG_WriteString((char*)pString);
	    DBG_WriteString("\n\r");
	}

	if(pBuf)
	{
            //print hex dump
	    DBG_HexDump(pBuf, buflen);
	    DBG_WriteString("\n\r");
	}

}

void DBG_PrintTimeStamp(void)
{
    uint64_t time =  (TMR_GetTimestamp()/1000);
    shell_printf("\n\rTime stamp ms: %lld\n\r", time);
}


 /*! *********************************************************************************
* \brief
* \param[in]
*
* \return
********************************************************************************** */
void DBG_MsgCheck(void)
{
#ifdef MEM_TRACKING
    uint16_t i, j;
    uint8_t* pHead;
    pools_t* pParentPool;

  OSA_EXT_InterruptDisable();
    for(i=0; i< poolCount; i++)
    {
        pHead = (uint8_t*)(memPools[i].anchor.head);
        if( (pHead != NULL) &&
            ((pHead < memHeap) || (pHead > (memHeap + heapSize)) )
          )
        {
            while(1);
        }
    }


    for(i=0; i<mTotalNoOfMsgs_d; i++)
    {
        pParentPool = (((listHeader_t *)(memTrack[i].blockAddr))-1)->pParentPool;
        for(j = 0; j < poolCount; j++)
        {
           if(pParentPool == &memPools[j])
           {
             break;
           }
        }
        if (j == poolCount)
        {
           while(1);
        }
    }
  OSA_EXT_InterruptEnable();
#endif
}


 /*! *********************************************************************************
* \brief     This function checks for buffer overflow when copying multiple bytes
*
* \param[in] p    - pointer to destination.
* \param[in] size - number of bytes to copy
*
* \return 1 if overflow detected, else 0
*
********************************************************************************** */
void DBG_MEMBufferCheck(uint8_t *p, uint32_t size)
{
#ifdef MEM_TRACKING
    uint32_t i;
    OSA_EXT_InterruptDisable();
    if( (p < (uint8_t*)memHeap) || (p > ((uint8_t*)memHeap + heapSize)) )
    {
        OSA_EXT_InterruptEnable();
        return;
    }
    for(i=0; i<mTotalNoOfMsgs_d; i++)
    {
        if( p > (uint8_t*)memTrack[i].blockAddr &&
            p < (uint8_t*)memTrack[i+1].blockAddr )
        {
            if( (p+size) > ((uint8_t*)memTrack[i+1].blockAddr - sizeof(listHeader_t)) )
            {
                panic(0,0,0,0);
            }

            break;
        }
    }
    OSA_EXT_InterruptEnable();
#endif
}


/* Delimiters */
/*================================================================================================*/

/*==================================================================================================
Private debug functions
==================================================================================================*/

#endif