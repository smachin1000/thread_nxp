/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
* \file FsciCommunication.c
* This is a source file for the FSCI communication.
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "FsciCommunication.h"
#include "FsciInterface.h"
#include "FunctionLib.h"

#include "MemManager.h"
#include "Panic.h"

#if gFsciIncluded_c
/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define gFsciUseBlockingTx_c 1
#define MIN_VALID_PACKET_LEN (sizeof(clientPacketHdr_t))
#define FSCI_txCallback MEM_BufferFree
#define FSCI_rxCallback FSCI_receivePacket

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
fsci_packetStatus_t FSCI_checkPacket( clientPacket_t *pData, uint16_t bytes, uint8_t* pVIntf );

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
uint8_t gFsciTxBlocking = FALSE;
uint8_t gFsciTxDisable  = FALSE;

/* Holds the SMGR interface Id associated with the FSCI interface (index) */
uint8_t gFsciInterfaces[gFsciMaxInterfaces_c];

#if gFsciMaxVirtualInterfaces_c
/* Holds the virtual interface Id for the FSCI interface (index) */
uint8_t gFsciVirtualInterfaces[gFsciMaxInterfaces_c];
#endif

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static fsciComm_t mFsciCommData[gFsciMaxInterfaces_c];

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  Initialize the serial interface.
*
* \param[in]  initStruct pointer to a gFsciSerialConfig_t structure
*
********************************************************************************** */
void FSCI_commInit( void* initStruct )
{
    uint32_t i;
    gFsciSerialConfig_t *pSerCfg = (gFsciSerialConfig_t*)initStruct;
#if gFsciMaxVirtualInterfaces_c
    uint32_t j;
#endif

    if (NULL == pSerCfg)
    {
        panic( ID_PANIC(0,0), (uint32_t)FSCI_commInit, 0, 0 );
        return;
    }

    FLib_MemSet( mFsciCommData, 0x00, sizeof(mFsciCommData) );

    for ( i = 0; i < gFsciMaxInterfaces_c; i++ )
    {
        gFsciInterfaces[i] = gSerialMgrInvalidIdx_c;

#if gFsciMaxVirtualInterfaces_c
        gFsciVirtualInterfaces[i] = pSerCfg[i].virtualInterface;

        if( pSerCfg[i].virtualInterface >= gFsciMaxInterfaces_c )
        {
            /* Error: invalid virtual interface */
            continue;
        }

        /* Check if the serial interface was allready initialized */
        for( j = 0; j < i; j++ )
        {
            if( pSerCfg[i].interfaceType == pSerCfg[j].interfaceType &&
                pSerCfg[i].interfaceChannel == pSerCfg[j].interfaceChannel )
            {
                gFsciInterfaces[i] = gFsciInterfaces[j];
            }
        }

        if( gFsciInterfaces[i] != gSerialMgrInvalidIdx_c )
        {
            /* The HW interface was allready initialized */
            continue;
        }
#endif
        Serial_InitInterface(&gFsciInterfaces[i],
                             pSerCfg[i].interfaceType,
                             pSerCfg[i].interfaceChannel );

        Serial_SetRxCallBack( gFsciInterfaces[i], FSCI_rxCallback, (void*)i);
        if( pSerCfg[i].interfaceType == gSerialMgrUart_c )
            Serial_SetBaudRate  ( gFsciInterfaces[i], pSerCfg[i].baudrate );
    }
}

/*! *********************************************************************************
* \brief  Receives data from the serial interface and checks to see if we have a valid pachet.
*
* \param[in]  param the fsciInterface on which he data has been received
*
********************************************************************************** */
void FSCI_receivePacket( void* param )
{
    uint8_t c;
    uint16_t readBytes;
    uint8_t virtualInterfaceId;
    fsci_packetStatus_t  status;
    fsciComm_t *pCommData = &mFsciCommData[(uint32_t)param];
    
    if ( gSerial_Success_c != Serial_GetByteFromRxBuffer( gFsciInterfaces[(uint32_t)param], &c, &readBytes ) )
        return;

    while( readBytes )
    {
        if( NULL == pCommData->pPacketFromClient )
        {
            pCommData->bytesReceived = 0;
            if( c == gFSCI_StartMarker_c )
            {
                pCommData->pPacketFromClient = (clientPacket_t*)&pCommData->pktHeader;
                pCommData->bytesReceived++;
            }
        }
        else
        {
            pCommData->pPacketFromClient->raw[pCommData->bytesReceived++] = c;

#if gFsciUseEscapeSeq_c
            FSCI_decodeEscapeSeq( pCommData->pPacketFromClient->raw, pCommData->bytesReceived );
#endif

            /* call the check pachet function to see if we have a valid packet */
            status = FSCI_checkPacket( pCommData->pPacketFromClient, pCommData->bytesReceived, &virtualInterfaceId );

            if( pCommData->bytesReceived == sizeof(clientPacketHdr_t) && status == PACKET_IS_TO_SHORT )
            {
                pCommData->pPacketFromClient = MEM_BufferAlloc( sizeof(clientPacketHdr_t) + pCommData->pktHeader.len + 1 );
                if( NULL != pCommData->pPacketFromClient )
                {
                    FLib_MemCpy(pCommData->pPacketFromClient, &pCommData->pktHeader, sizeof(clientPacketHdr_t));
                }
            }

            if( status == PACKET_IS_VALID )
            {
                /* Send the message to Test Kernel*/
                FSCI_ProcessRxPkt (pCommData->pPacketFromClient, FSCI_GetFsciInterface((uint32_t)param, virtualInterfaceId));
                pCommData->pPacketFromClient = NULL;
            }
            else if (status == FRAMING_ERROR)
            {
#if 0
                uint16_t i;

                /* If there appears to be a framing error, search the data received for */
                /* the next STX and try again. */
                c = 0;
                for (i = 0; i < pCommData->bytesReceived; ++i)
                {
                    if (pCommData->pPacketFromClient->raw[i] == gFSCI_StartMarker_c)
                    {
                        c = gFSCI_StartMarker_c;
                        pCommData->bytesReceived -= i;
                        FLib_MemCpy( pCommData->pPacketFromClient->raw,
                                    pCommData->pPacketFromClient->raw + i,
                                    pCommData->bytesReceived);
#if 0
                        /* If a payload buffer was alocated, and the new payload is larger than the buffer's size,
                        * then a new buffer must be allocated. The received data is copied into the new buffer,
                        * and the old one is freed.
                        */
                        if( (void*)pCommData->pPacketFromClient != (void*)&pCommData->pktHeader &&
                            pCommData->bytesReceived >= sizeof(clientPacketHdr_t) &&
                            pCommData->pPacketFromClient->structured.header.len > MEM_BufferGetSize(pCommData->pPacketFromClient) 
                          )
                        {
                            clientPacket_t *p;
                            p = MEM_BufferAlloc( sizeof(clientPacketHdr_t) + pCommData->pPacketFromClient->structured.header.len + 1 );
                            
                            if( NULL != p )
                            {
                                FLib_MemCpy(p->raw,
                                            pCommData->pPacketFromClient->raw,
                                            pCommData->bytesReceived);
                            }
                            
                            MEM_BufferFree( pCommData->pPacketFromClient );                                
                            pCommData->pPacketFromClient = p;
                        }
#endif
                        break;
                    }
                }

                if( c != gFSCI_StartMarker_c )
#endif
                {
                    if( pCommData->pPacketFromClient != (clientPacket_t*)&pCommData->pktHeader )
                        MEM_BufferFree(pCommData->pPacketFromClient);

                    pCommData->pPacketFromClient = NULL;
                }
                
            } /* if (status == FRAMING_ERROR) */
            
        }  /* if (!startOfFrameSeen) */

        if ( gSerial_Success_c != Serial_GetByteFromRxBuffer( gFsciInterfaces[(uint32_t)param], &c, &readBytes ) )
            return;

    } /* while (j < size) */
}

/*! *********************************************************************************
* \brief  Send packet over the serial interface, after computing Checksum.
*
* \param[in] pPacket pointer to the packet to be sent over the serial interface
* \param[in] fsciInterface the interface on which the packet should be sent
*
********************************************************************************** */
void FSCI_transmitFormatedPacket( void *pPacket, uint32_t fsciInterface )
{
    clientPacket_t *pPkt = pPacket;
    uint32_t size;
    uint8_t checksum;
    uint32_t virtInterface = FSCI_GetVirtualInterface(fsciInterface);

    pPkt->structured.header.startMarker = gFSCI_StartMarker_c;
    size = sizeof(clientPacketHdr_t) + pPkt->structured.header.len + 1 /* CRC */;

    /* Compute Checksum */
    checksum = FSCI_computeChecksum( pPkt->raw+1, size - 2);
    
    pPkt->structured.payload[pPkt->structured.header.len] = checksum;
    
    if( virtInterface )
    {
#if gFsciMaxVirtualInterfaces_c
        pPkt->structured.payload[pPkt->structured.header.len] += virtInterface;
        pPkt->structured.payload[pPkt->structured.header.len+1] = checksum^(checksum + virtInterface);
        size += sizeof(checksum);
#else
       (void)virtInterface;
#endif
    }

    /* send message to Serial Manager */
#if gFsciUseBlockingTx_c
    if ( gFsciTxBlocking )
    {
        Serial_SyncWrite( gFsciInterfaces[fsciInterface], pPkt->raw, size );
        MEM_BufferFree( pPkt );
    }
    else
#endif
    Serial_AsyncWrite( gFsciInterfaces[fsciInterface], pPkt->raw, size, (pSerialCallBack_t)FSCI_txCallback, pPkt );
}

/*! *********************************************************************************
* \brief  Encode and send messages over the serial interface
*
* \param[in] OG operation Group
* \param[in] OC operation Code
* \param[in] pMsg pointer to payload
* \param[in] msgLen length of the payload
* \param[in] fsciInterface the interface on which the packet should be sent
*
********************************************************************************** */
void FSCI_transmitPayload( uint8_t OG, uint8_t OC, void *pMsg, uint16_t msgLen, uint32_t fsciInterface )
{
    uint8_t* buffer_ptr = NULL;
    uint16_t buffer_size, index;
    uint8_t checksum, checksum2;
    clientPacketHdr_t header;
    uint32_t virtInterface = FSCI_GetVirtualInterface(fsciInterface);

    if( gFsciTxDisable || msgLen > gFsciMaxPayloadLen_c )
        return;

    /* Compute size */
    buffer_size = sizeof(clientPacketHdr_t) + msgLen + 2*sizeof(checksum);

#if gFsciUseEscapeSeq_c
    buffer_size = buffer_size*2;
#endif

    /* Allocate buffer */
    buffer_ptr = MEM_BufferAlloc( buffer_size );
    if( NULL == buffer_ptr )
    {
        panic( ID_PANIC(0,0), (uint32_t)FSCI_transmitPayload, 0, 0 );
        return;
    }

    /* Message header */
    header.startMarker = gFSCI_StartMarker_c;
    header.opGroup = OG;
    header.opCode = OC;
    header.len = msgLen;

    /* Compute CRC for TX packet, on opcode group, opcode, payload length, and payload fields */
    checksum = FSCI_computeChecksum((uint8_t*)&header + 1, sizeof(header) - 1);
    checksum ^= FSCI_computeChecksum((uint8_t*)pMsg, msgLen);
    if( virtInterface )
    {
        checksum2 = checksum^(checksum + virtInterface);
        checksum += virtInterface;
    }

    index = 0;
#if gFsciUseEscapeSeq_c
    index += FSCI_encodeEscapeSeq( (uint8_t*)&header, sizeof(header), &buffer_ptr[index] );
    index += FSCI_encodeEscapeSeq( pMsg, msgLen, &buffer_ptr[index]);
    /* Store the Checksum*/
    index += FSCI_encodeEscapeSeq( (uint8_t*)&checksum, sizeof(checksum), &buffer_ptr[index] );
    if( virtInterface )
    {
        index += FSCI_encodeEscapeSeq( (uint8_t*)&checksum2, sizeof(checksum2), &buffer_ptr[index] );
    }
    buffer_ptr[index++] = gFSCI_EndMarker_c;

#else /* gFsciUseEscapeSeq_c */
    FLib_MemCpy( &buffer_ptr[index], &header, sizeof(header) );
    index += sizeof(header);
    FLib_MemCpy( &buffer_ptr[index], pMsg, msgLen );
    index += msgLen;
    /* Store the Checksum */
    buffer_ptr[index++] = checksum;
    if( virtInterface )
    {
        buffer_ptr[index++] = checksum2;
    }
    
#endif /* gFsciUseEscapeSeq_c */

    /* send message to Serial Manager */
#if gFsciUseBlockingTx_c
    if ( gFsciTxBlocking )
    {
        Serial_SyncWrite( gFsciInterfaces[fsciInterface], buffer_ptr, index );
        MEM_BufferFree( buffer_ptr );
    }
    else
#endif
    {
        Serial_AsyncWrite( gFsciInterfaces[fsciInterface], buffer_ptr, index, (pSerialCallBack_t)FSCI_txCallback, buffer_ptr );
    }
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  Returnd the virtual interface associated with the specified fsciInterface.
*
* \param[in] fsciInterface the interface on which the packet should be sent
*
* \return the Virtual Interface Id
*
********************************************************************************** */
uint32_t FSCI_GetVirtualInterface(uint32_t fsciInterface)
{
#if gFsciMaxVirtualInterfaces_c
    return gFsciVirtualInterfaces[fsciInterface];
#else
    return 0;
#endif
}

/*! *********************************************************************************
* \brief  Determines the FSCI interface for a received packet.
*
* \param[in] hwInterface the serial interface on which the packet was received.
* \param[in] virtualInterface the virtual interface of the received packet
*
* \return the FSCI interface
*
********************************************************************************** */
uint32_t FSCI_GetFsciInterface(uint32_t hwInterface, uint32_t virtualInterface)
{
#if gFsciMaxVirtualInterfaces_c
    uint32_t i;
    
    for( i = 0; i < gFsciMaxInterfaces_c; i++)
    {
        if( virtualInterface == gFsciVirtualInterfaces[i] &&
            hwInterface == gFsciInterfaces[i] )
            return i;
    }
#endif
    return hwInterface;
}

/*! *********************************************************************************
* \brief  Checks to see if we have a valid packet
*
* \param[in] pData The message containing the incoming data packet to be handled.
* \param[in] bytes the number of bytes inside the buffer
* \param[Out] pVIntf pointer to the location where the virtual interface Id will be stored
*
* \return the status of the packet
*
********************************************************************************** */
fsci_packetStatus_t FSCI_checkPacket( clientPacket_t *pData, uint16_t bytes, uint8_t* pVIntf )
{
    uint8_t checksum = 0;
    uint16_t len;

    if ( bytes < MIN_VALID_PACKET_LEN )
    {
        return PACKET_IS_TO_SHORT;            /* Too short to be valid. */
    }

    if ( bytes >= sizeof(clientPacket_t) )
    {
        return FRAMING_ERROR;
    }

    if ( NULL == pData )
    {
        return INTERNAL_ERROR;
    }

    /* The packet's len field does not count the STX, the opcode group, the */
    /* opcode, the len field, or the checksum. */
    len = pData->structured.header.len;
    
    /* If the length appears to be too long, it might be because the external */
    /* client is sending a packet that is too long, or it might be that we're */
    /* out of sync with the external client. Assume we're out of sync. */
    if ( len > sizeof(pData->structured.payload) )
    {
        return FRAMING_ERROR;
    }    
    
    if ( bytes < len + sizeof(clientPacketHdr_t) + sizeof(checksum) )
    {
        return PACKET_IS_TO_SHORT;
    }

    /* If the length looks right, make sure that the checksum is correct. */
    if( bytes >= len + sizeof(clientPacketHdr_t) + sizeof(checksum) )
    {
        checksum = FSCI_computeChecksum(pData->raw+1, len + sizeof(clientPacketHdr_t)-1);
        *pVIntf = pData->structured.payload[len] - checksum;
    }
    
    if( bytes == len + sizeof(clientPacketHdr_t) + sizeof(checksum) )
    {
        if( 0 == *pVIntf )
        {
            return PACKET_IS_VALID;
        }
#if gFsciMaxVirtualInterfaces_c
        else if( *pVIntf < gFsciMaxVirtualInterfaces_c )
        {
            return PACKET_IS_TO_SHORT;
        }
#endif
    }

#if gFsciMaxVirtualInterfaces_c
    /* Check virtual interface */
    if( bytes == len + sizeof(clientPacketHdr_t) + 2*sizeof(checksum) )
    {
        checksum ^= checksum + *pVIntf;
        if( pData->structured.payload[len+1] == checksum )
        {
            return PACKET_IS_VALID;
        }
    }
#endif

    return FRAMING_ERROR;
}

/*! *********************************************************************************
* \brief  This function performs a XOR over the message to compute the CRC
*
* \param[in]  pBuffer - pointer to the messae
* \param[in]  size - the length of the message
*
* \return  the CRC of the message
*
********************************************************************************** */
uint8_t FSCI_computeChecksum( void *pBuffer, uint16_t size )
{
    uint16_t index;
    uint8_t  checksum = 0;

    for ( index = 0; index < size; index++ )
    {
        checksum ^= ((uint8_t*)pBuffer)[index];
    }

    return checksum;
}

/*! *********************************************************************************
* \brief  This function performs the encoding of a message, using the Escape Sequence
*
* \param[in]  pDataIn, pointer to the messae to be encoded
* \param[in]  len, the length of the message
* \param[out]  pDataOut, pointer to the encoded message
*
* \return  The number of bytes added in the new buffer
*
********************************************************************************** */
#if gFsciUseEscapeSeq_c
uint32_t FSCI_encodeEscapeSeq( uint8_t* pDataIn, uint32_t len, uint8_t* pDataOut )
{
    uint32_t index, new_index = 0;

    if( NULL != pDataOut )
    {
        for ( index = 0; index < len; index++ )
        {
            if( (pDataIn[index] == gFSCI_StartMarker_c) ||
               (pDataIn[index] == gFSCI_EndMarker_c)    ||
                   (pDataIn[index] == gFSCI_EscapeChar_c) )
            {
                pDataOut[new_index++] = gFSCI_EscapeChar_c;
                pDataOut[new_index++] = pDataIn[index] ^ gFSCI_EscapeChar_c;
            }
            else
            {
                pDataOut[new_index++] = pDataIn[index];
            }
        }
    }

    return new_index;
}
#endif

/*! *********************************************************************************
* \brief  This function performs the decoding of a message, using the Escape Sequence
*
* \param[in]  pData pointer to the messae to be encoded
* \param[in]  len the length of the message
*
*
********************************************************************************** */
#if gFsciUseEscapeSeq_c
void FSCI_decodeEscapeSeq( uint8_t* pData, uint32_t len )
{
    uint32_t index, new_index;

    /* Find the first gFSCI_EscapeChar_c */
    for ( index = 0; index < len; index++ )
        if ( pData[index] == gFSCI_EscapeChar_c )
            break;

    new_index = index;

    /* If a gFSCI_EscapeChar_c was found, decode the packet in place */
    while ( index < len )
    {
        if ( pData[index] == gFSCI_EscapeChar_c )
        {
            index++; /* skip over the gFSCI_EscapeChar_c */

            if ( index < len )
                pData[new_index++] = pData[index++] ^ gFSCI_EscapeChar_c;
        }
        else if ( new_index != index )
        {
            pData[new_index++] = pData[index++];
        }
    }
}
#endif

#endif /* gFsciIncluded_c */
