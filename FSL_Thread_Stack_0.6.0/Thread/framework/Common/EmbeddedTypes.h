/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file EmbeddedTypes.h
* This file holds type definitions that maps the standard c-types into types
* with guaranteed sizes. The types are target/platform specific and must be edited
* for each new target/platform.
* The header file also provides definitions for TRUE, FALSE and NULL.
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

#ifndef _EMBEDDEDTYPES_H_
#define _EMBEDDEDTYPES_H_


/************************************************************************************
*
*       INCLUDES
*
************************************************************************************/

#include <stdint.h>


/************************************************************************************
*
*       TYPE DEFINITIONS
*
************************************************************************************/

/* boolean types */
typedef uint8_t   bool_t;

typedef uint8_t    index_t;

/* TRUE/FALSE definition*/
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* null pointer definition*/
#ifndef NULL
#define NULL (( void * )( 0x0UL ))
#endif

#if defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__ ((__packed__))
#define PACKED_UNION  union __attribute__ ((__packed__))
#elif defined(__IAR_SYSTEMS_ICC__)
#define PACKED_STRUCT __packed struct
#define PACKED_UNION __packed union
#else
#define PACKED_STRUCT struct
#define PACKED_UNION union
#endif

typedef unsigned char uintn8_t;
typedef unsigned long uintn32_t;

#define MIN(a,b)                    (((a) < (b))?(a):(b))

/* Compute the number of elements of an array */
#define NumberOfElements(x) (sizeof(x)/sizeof((x)[0]))

#define GetRelAddr(strct, member) ((uint32_t)&(((strct*)(void *)0)->member))
#define GetSizeOfMember(strct, member) sizeof(((strct*)(void *)0)->member)

/* Type definitions for link configuration of instantiable layers  */
#define gInvalidInstanceId_c (instanceId_t)(-1)
typedef uint32_t instanceId_t;


#endif /* _EMBEDDEDTYPES_H_ */
