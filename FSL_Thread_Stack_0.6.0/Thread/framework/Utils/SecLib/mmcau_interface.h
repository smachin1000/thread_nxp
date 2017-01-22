/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file mmcau_interface.h
* This is the header file for MMCAU hardware module
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
*
*/

#ifndef _MMCAU_INTERFACE_H_
#define _MMCAU_INTERFACE_H_

#include "EmbeddedTypes.h"

/* AES constants */
#define AES128 128
#define AES128_ROUNDS 10

#define AES192 192
#define AES192_ROUNDS 12

#define AES256 256
#define AES256_ROUNDS 14

/* NOTE: All Input/Output pointers must refer to a 4 bytes aligned memory address! */

/* AES functions */
extern void mmcau_aes_set_key (const uint8_t* key, const int32_t key_size, uint8_t* key_sch);
extern void mmcau_aes_encrypt (const uint8_t* in, const uint8_t* key_sch,  const int32_t nr, uint8_t* out);
extern void mmcau_aes_decrypt (const uint8_t* in, const uint8_t* key_sch,  const int32_t nr, uint8_t* out);

/* DES functions */
extern int32_t mmcau_des_chk_parity (const uint8_t* key);
extern void mmcau_des_encrypt (const uint8_t* in, const uint8_t* key, uint8_t* out);
extern void mmcau_des_decrypt (const uint8_t* in, const uint8_t* key, uint8_t* out);

/* MD5 Functions */
extern void mmcau_md5_initialize_output (const uint8_t* md5_state);
extern void mmcau_md5_hash_n (const uint8_t* msg_data, const int32_t num_blks, uint8_t* md5_state);
extern void mmcau_md5_update (const uint8_t* msg_data, const int32_t num_blks, uint8_t* md5_state);
extern void mmcau_md5_hash   (const uint8_t* msg_data, uint8_t* md5_state);

/* SHA1 Functions */
extern void mmcau_sha1_initialize_output (const uint32_t* sha1_state);
extern void mmcau_sha1_hash_n (const uint8_t* msg_data, const int32_t num_blks, uint32_t* sha1_state);
extern void mmcau_sha1_update (const uint8_t* msg_data, const int32_t num_blks, uint32_t* sha1_state);
extern void mmcau_sha1_hash   (const uint8_t* msg_data, uint32_t* sha1_state);

/* SHA256 Functions */
extern int32_t mmcau_sha256_initialize_output (const uint32_t* output);
extern void mmcau_sha256_hash_n (const uint8_t* input, const int32_t num_blks, uint32_t* output);
extern void mmcau_sha256_update (const uint8_t* input, const int32_t num_blks, uint32_t* output);
extern void mmcau_sha256_hash   (const uint8_t* input, uint32_t* output);


#endif /* _MMCAU_INTERFACE_H_ */