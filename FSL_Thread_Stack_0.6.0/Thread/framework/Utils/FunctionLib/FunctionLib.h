/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file FunctionLib.h
* This is the Function Lib module header file
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

#ifndef _FUNCTION_LIB_H_
#define _FUNCTION_LIB_H_

#include "EmbeddedTypes.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

#ifndef gUseToolchainMemFunc_d
#define gUseToolchainMemFunc_d 0
#endif

#define FLib_MemSet16 FLib_MemSet

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************************
* Copy the content of one memory block to another. The amount of data to copy must
* be specified in number of bytes.
*
* Interface assumptions:
*   None
*
* Return value:
*   None
*
************************************************************************************/

void FLib_MemCpy (void* pDst,     /* IN: Pointer to destination memory block */
                  void* pSrc,     /* IN: Pointer to source memory block */
                  uint32_t cBytes /* IN: Number of bytes to copy */
                  );


void FLib_MemCpyAligned32bit (void* to_ptr,
                              void* from_ptr,
                              register uint32_t number_of_bytes);


void FLib_MemCpyDir (void* pBuf1,
                     void* pBuf2,
                     bool_t dir,
                     uint32_t n);


/************************************************************************************
* Copy up to 255 bytes. The byte at index i from the source buffer is copied to index
* ((n-1) - i) in the destination buffer (and vice versa).
*
* Interface assumptions:
*   None
*
* Return value:
*   None
*
************************************************************************************/
void FLib_MemCpyReverseOrder (void* pDst,      /* Destination buffer */
                              void* pSrc,      /* Source buffer */
                              uint32_t cBytes  /* Byte count */
                             );


/************************************************************************************
* Compare two memory blocks. The number of bytes to compare must be specified. If the
* blocks are equal byte by byte, the function returns TRUE, and FALSE otherwise.
*
* Interface assumptions:
*   None
*
* Return value:
*   TRUE if memory areas are equal. FALSE othwerwise.
*
************************************************************************************/
bool_t FLib_MemCmp (void* pData1,   /* IN: First memory block to compare */
                    void* pData2,   /* IN: Second memory block to compare */
                    uint32_t cBytes /* IN: Number of bytes to compare. */
                   );


/************************************************************************************
* Reset bytes in a memory block to a certain value. The value, and the number of
* bytes o be set, are supplied as arguments.
*
* Interface assumptions:
*   None
*
* Return value:
*   None
*
************************************************************************************/
 void FLib_MemSet (void* pData,     /* IN: Pointer to memory block to reset */
                   uint8_t value,   /* IN: Value that memory block will be reset to. */
                   uint32_t cBytes  /* IN: Number of bytes to reset. */
                  );


/************************************************************************************
* Copy up to 256 bytes, possibly into the same overlapping memory as it is taken from
*
* Interface assumptions:
*   None
*
* Return value:
*   None
*
************************************************************************************/
void FLib_MemInPlaceCpy (void* pDst,      /* Destination buffer */
                         void* pSrc,      /* Source buffer */
                         uint32_t cBytes  /* Byte count */
                        );


/************************************************************************************
* Copies a 16bit value to an unaligned a memory block.
*
* Interface assumptions:
*   None
*
* Return value:
*   None
*
************************************************************************************/
void FLib_MemCopy16Unaligned (void* pDst,      /* Pointer to destination memory block */
                              uint16_t val16   /* The value to be copied */
                             );


/************************************************************************************
* Copies a 32bit value to an unaligned a memory block.
*
* Interface assumptions:
*   None
*
* Return value:
*   None
*
************************************************************************************/
void FLib_MemCopy32Unaligned (void* pDst,      /* Pointer to destination memory block */
                              uint32_t val32   /* The value to be copied */
                             );


/************************************************************************************
* Copies a 64bit value to an unaligned a memory block.
*
* Interface assumptions:
*   None
*
* Return value:
*   None
*
************************************************************************************/
void FLib_MemCopy64Unaligned (void* pDst,     /* Pointer to destination memory block */
                              uint64_t val64  /* The value to be copied */
                             );


/************************************************************************************
* Add an offset to a pointer. The offset can be in the range 0-255.
*
* Interface assumptions:
*   None
*
* Return value:
*    None
*
************************************************************************************/
void FLib_AddOffsetToPointer (void** pPtr,
                              uint32_t offset);

#define FLib_AddOffsetToPtr(pPtr,offset) FLib_AddOffsetToPointer((void**)(pPtr),(offset))


/************************************************************************************
* Compare two bytes.
*
* Interface assumptions:
*   None
*
* Return value:
*   TRUE if content of buffers is equal, and FALSE otherwise.
*
*
************************************************************************************/
#define FLib_Cmp2Bytes(c1, c2) (*((uint16_t*) (c1)) == *((uint16_t*) (c2)))


/************************************************************************************
* Returns the maximum value of arguments a and b.
*
* Interface assumptions:
*   The primitive should must be implemented as a macro, as it should be possible to
*   evaluate the result on compile time if the arguments are constants.
*
* Return value:
*   The maximum value of arguments a and b
*
************************************************************************************/
#define FLib_GetMax(a,b)    (((a) > (b)) ? (a) : (b))


/************************************************************************************
* Returns the minimum value of arguments a and b.
*
* Interface assumptions:
*   The primitive should must be implemented as a macro, as it should be possible to
*   evaluate the result on compile time if the arguments are constants.
*
************************************************************************************/
#define FLib_GetMin(a,b)    (((a) < (b)) ? (a) : (b))

#ifdef __cplusplus
}
#endif

#endif /* _FUNCTION_LIB_H_ */
