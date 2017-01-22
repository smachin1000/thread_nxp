/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file NV_Flash.c
* Implementation of the non-volatile storage module for the CORTEX-M4 processor
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

#include "EmbeddedTypes.h"
#include "NV_Flash.h"
#include "NV_FsciCommands.h"
#include "TimersManager.h"
#include "RNG_Interface.h"
#include "FunctionLib.h"

#include "fsl_os_abstraction.h"
#include "Flash_Adapter.h"

#if (gFsciIncluded_c && (gNvmEnableFSCIRequests_c || gNvmEnableFSCIMonitoring_c))
#include "NV_FsciCommands.h"
#endif

#include "Messaging.h"
/*****************************************************************************
 *****************************************************************************
 * Private macros
 *****************************************************************************
 *****************************************************************************/
#if gNvStorageIncluded_d

/*
 * Name: gNvVirtualPagesCount_c
 * Description: the count of virtual pages used
 */
#define gNvVirtualPagesCount_c         2 /* DO NOT MODIFY */


 #if (gUnmirroredFeatureSet_d == TRUE)
   #if (gNvUseFlexNVM_d == TRUE)
     #error "*** ERROR: gUnmirroredFeatureSet_d not implemented on FlexNVM"
   #endif
   #if (gNvFragmentation_Enabled_d == FALSE)
     #error "*** ERROR: gNvFragmentation_Enabled_d should be enabled for gUnmirroredFeatureSet_d"
   #endif
   #if (gNvUseExtendedFeatureSet_d == FALSE)
     #error "*** ERROR: gNvUseExtendedFeatureSet_d should be enabled for gUnmirroredFeatureSet_d"
   #endif
 #endif

#endif /* gNvStorageIncluded_d */
/*****************************************************************************
 *****************************************************************************
 * Private type definitions
 *****************************************************************************
 *****************************************************************************/
#if PGM_SIZE_BYTE == FTFx_PHRASE_SIZE
typedef uint64_t NV_baseType;
#else
typedef uint32_t NV_baseType;
#endif

/*****************************************************************************
 *****************************************************************************
 * Private prototypes
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d

/******************************************************************************
 * Name: __NvRegisterTableEntry
 * Description: The function tries to register a new table entry within an
 *              existing NV table. If the NV table contained an erased (invalid)
 *              entry, the entry will be overwritten with a new one (provided
 *              by the mean of this function arguments)      
 * Parameter(s): [IN] ptrData - generic pointer to RAM data to be registered
 *                              within the NV storage system
 *               [IN] uniqueId - an unique ID of the table entry
 *               [IN] elemCount - how many elements the table entry contains
 *               [IN] elemSize - the size of an element 
 *               [IN] overwrite - if an existing table entry shall be 
 *                                overwritten
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *****************************************************************************/
#if gNvUseExtendedFeatureSet_d   
static NVM_Status_t __NvRegisterTableEntry
(
  void* ptrData,
  NvTableEntryId_t uniqueId,
  uint16_t elemCount,
  uint16_t elemSize,
  bool_t overwrite
);
#endif /* gNvUseExtendedFeatureSet_d */

/******************************************************************************
 * Name: __NvAtomicSave
 * Description: The function performs an atomic save of the entire NV table
 *              to the storage system. The operation is performed
 *              in place (atomic).
 * Parameter(s):  [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
static NVM_Status_t __NvAtomicSave
(
  bool_t ignoreCriticalSectionFlag
);

/******************************************************************************
 * Name: __NvSyncSave
 * Description: The function saves the pointed element or the entire table
 *              entry to the storage system. The save operation is not
 *              performed on the idle task but within this function call.
 * Parameter(s): [IN] ptrData - a pointer to data to be saved
 *               [IN] saveAll - specifies if the entire table entry shall be 
 *                              saved or only the pointed element
 *               [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section                             
 *****************************************************************************/
static NVM_Status_t __NvSyncSave
(
  void* ptrData,
  bool_t saveAll,
  bool_t ignoreCriticalSectionFlag
);
/******************************************************************************
 * Name: __NvEraseEntryFromStorage
 * Description: The function removes a table entry within the existing NV 
 *              table. 
 * Parameter(s): [IN] ptrData - a pointer to an existing RAM data that is
 *                              managed by the NV storage system    
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *****************************************************************************/
#if gNvUseExtendedFeatureSet_d
static NVM_Status_t __NvEraseEntryFromStorage
(
  void* ptrData
);
#endif /* gNvUseExtendedFeatureSet_d */
/******************************************************************************
 * Name: __NvFormat
 * Description: Format the NV storage system. The function erases both virtual
 *              pages and then writes the page counter to active page.              
 * Parameter(s): -
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_CriticalSectionActive_c - if the system has entered in a 
 *                                        critical section
 *****************************************************************************/
static NVM_Status_t __NvFormat
(
  void
);
/******************************************************************************
 * Name: __NvIdle
 * Description: Called from the idle task (bare-metal) or NVM_Task (MQX,
 *              FreeRTOS) to process the pending saves, erase or copy 
 *              operations.
 * Parameters: -
 * Return: -
 ******************************************************************************/
static void __NvIdle
(
  void
);
/******************************************************************************
 * Name: __NvRestoreDataSet
 * Description: copy the most recent version of the element/table entry pointed 
 *              by ptrData from NVM storage system to RAM memory
 * Parameter(s): [IN] ptrData - pointer to data (element) to be restored 
 *               [IN] restoreAll - if FALSE restores a single element
 *                               - if TRUE restores an entire table entry
 * Return: status of the restore operation
 *****************************************************************************/
static NVM_Status_t __NvRestoreDataSet
(
  void* ptrData,    
  bool_t restoreAll
);

/******************************************************************************
 * Name: __NvTimerTick
 * Description: Called from the idle task to process save-on-interval requests
 * Parameters: [IN] countTick - enable/disable tick count
 * Return: FALSE if the timer tick counters for all data sets have reached 
 *         zero. In this case, the timer can be turned off.
 *         TRUE if any of the data sets' timer tick counters have not yet
 *         counted down to zero. In this case, the timer should be active
 ******************************************************************************/
static bool_t __NvTimerTick
(
  bool_t countTick
);
/******************************************************************************
 * Name: __NvSaveOnCount
 * Description: Decrement the counter. Once it reaches 0, the next call to 
 *              NvIdle() will save the entire table entry (all elements).
 * Parameters: [IN] ptrData - pointer to data to be saved
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function 
 ******************************************************************************/
static NVM_Status_t __NvSaveOnCount
(
  void* ptrData
);
/******************************************************************************
 * Name: __NvSaveOnInterval
 * Description:  save no more often than a given time interval. If it has 
 *               been at least that long since the last save,
 *               this function will cause a save the next time the idle 
 *               task runs.
 * Parameters: [IN] ptrData - pointer to data to be saved
 * NOTE: this function saves all the element of the table entry pointed by
 *       ptrData 
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function 
 ******************************************************************************/
static NVM_Status_t __NvSaveOnInterval
(
  void* ptrData
);
/******************************************************************************
 * Name: __NvSaveOnIdle
 * Description: Save the data pointed by ptrData on the next call to NvIdle()
 * Parameter(s): [IN] ptrData - pointer to data to be saved
 *               [IN] saveRestoreAll - specify if all the elements from the NVM table
 *                              entry shall be saved
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_Error_c - in case of error(s)
 *         Note: see also return codes of NvGetEntryFromDataPtr() function         
 ******************************************************************************/
static NVM_Status_t __NvSaveOnIdle
(
  void* ptrData,
  bool_t saveAll
);   

/******************************************************************************
 * Name: __NvModuleInit
 * Description: Initialize the NV storage module
 * Parameter(s): -
 * Return: gNVM_ModuleAlreadyInitialized_c - if the module is already 
 *                                           initialized
 *         gNVM_InvalidSectorsCount_c - if the sector count configured in the
 *                                      project linker file is invalid
 *         gNVM_MetaNotFound_c - if no meta information was found                                       
 *         gNVM_OK_c - module was successfully initialized
 *****************************************************************************/
static NVM_Status_t __NvModuleInit
(
  void
);

/******************************************************************************
 * Name: __NvmMoveToRam
 * Description: Move from NVM to Ram
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: pointer to Ram location
 *****************************************************************************/
#if gUnmirroredFeatureSet_d
static NVM_Status_t __NvmMoveToRam
(
  void** ppData
);
#endif
/******************************************************************************
 * Name: __NvmErase
 * Description: Erase from NVM an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be erased
 * Return: pointer to Ram location
 *****************************************************************************/
#if gUnmirroredFeatureSet_d
static NVM_Status_t __NvmErase
(
  void** ppData
);
#endif
/******************************************************************************
 * Name: NvInitPendingSavesQueue
 * Description: Initialize the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 * Return: TRUE if the pointer is valid, FALSE otherwise
 ******************************************************************************/
static bool_t NvInitPendingSavesQueue
(
  NVM_SaveQueue_t *pQueue
);


/******************************************************************************
 * Name: NvPushPendingSave
 * Description: Add a new pending save to the queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [IN] data - data to be saved
 * Return: TRUE if the push operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPushPendingSave
(
  NVM_SaveQueue_t *pQueue, 
  NVM_TableEntryInfo_t data    
);


/******************************************************************************
 * Name: NvPopPendingSave
 * Description: Retrieves the head element from the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [OUT] pData - pointer to the location where data will be placed
 * Return: TRUE if the pop operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPopPendingSave
(
  NVM_SaveQueue_t *pQueue, 
  NVM_TableEntryInfo_t *pData
);

/******************************************************************************
 * Name: NvGetPendingSavesCount
 * Description: self explanatory
 * Parameters: [IN] pQueue - pointer to queue
 * Return: Number of pending saves
 ******************************************************************************/
static uint8_t NvGetPendingSavesCount
(
  NVM_SaveQueue_t *pQueue 
);

/*****************************************************************
 * The below functions are compiled only if FlexNVM is NOT used 
 *****************************************************************/

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/******************************************************************************
 * Name: NvEraseVirtualPage
 * Description: erase the specified page
 * Parameter(s): [IN] pageID - the ID of the page to be erased
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid
 *         gNVM_SectorEraseFail_c - if the page cannot be erased
 *         gNVM_OK_c - if operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvEraseVirtualPage
(
  NVM_VirtualPageID_t pageID
);


/******************************************************************************
 * Name: NvInitStorageSystem
 * Description: Initialize the storage system, retrieve the active page and
 *              the page counter. Called once by NvModuleInit() function.
 * Parameter(s): - 
 * Return: -
 *****************************************************************************/
static void NvInitStorageSystem
(
  void
);


/******************************************************************************
 * Name: NvVirtualPageBlankCheck
 * Description: checks if the specified page is blank (erased) 
 * Parameter(s): [IN] pageID - the ID of the page to be checked
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid 
 *         gNVM_PageIsNotBlank_c - if the page is not blank
 *         gNVM_OK_c - if the page is blank (erased)
 *****************************************************************************/
static NVM_Status_t NvVirtualPageBlankCheck
(
  NVM_VirtualPageID_t pageID
);


/******************************************************************************
 * Name: NvGetLastMetaInfoAddress
 * Description: retrieve and store (update) the last meta information address 
 * Parameter(s): -
 * Return: gNVM_MetaNotFound_c - if no meta information has been found
 *         gNVM_OK_c - if the meta was found and stored (updated)
 *****************************************************************************/
static NVM_Status_t NvGetLastMetaInfoAddress
(    
  void
);


/******************************************************************************
 * Name: NvGetMetaInfo
 * Description: get meta information based on the meta information address
 * Parameter(s): [IN] pageID - the ID of the page
 *               [IN] metaInfoAddress - meta information address
 *               [OUT] pMetaInfo - a pointer to a memory location where the 
 *                                 requested meta information will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_AddressOutOfRange_c - if the provided address is out of range
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetMetaInfo
(
  NVM_VirtualPageID_t pageId,
  uint32_t metaInfoAddress,
  NVM_RecordMetaInfo_t* pMetaInfo
);


/******************************************************************************
 * Name: NvGetPageFreeSpace
 * Description: return the page free space, in bytes
 * Parameter(s): [OUT] ptrFreeSpace - a pointer to a memory location where the
 *                                    page free space will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if the page is empty
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetPageFreeSpace
(    
  uint32_t* ptrFreeSpace
);


/******************************************************************************
 * Name: NvIsMemoryAreaAvailable
 * Description: checks if the specified memory area is blank (erased)
 * Parameter(s): [IN] address - start address
 *               [IN] len - length to be verified
 * Return: TRUE if the area is available (blank), FALSE otherwise
 *****************************************************************************/
static bool_t NvIsMemoryAreaAvailable
(
  uint32_t address,
  uint32_t len
);


/******************************************************************************
 * Name: NvIsRecordCopied
 * Description: Checks if a record or an entire table entry is already copied. 
 *              Called by page copy function.
 * Parameter(s): [IN] pageId - the ID of the page where to perform the check
 *               [IN] metaInf - a pointer to source page meta information tag
 * Return: TRUE if the element is already copied, FALSE otherwise
 *****************************************************************************/
static bool_t NvIsRecordCopied
(
  NVM_VirtualPageID_t pageId,
  NVM_RecordMetaInfo_t* metaInf
);


/******************************************************************************
 * Name: NvInternalCopy
 * Description: Performs a copy of an record / entire table entry
 * Parameter(s): [IN] dstAddress - destination record address
 *               [IN] dstMetaAddress - destination meta address
 *               [IN] srcMetaInfo - source meta information
 *               [IN] srcTblEntryIdx - source table entry index
 *               [IN] size - bytes to copy
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not 
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be 
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvInternalCopy
(
  uint32_t dstAddress,
  uint32_t dstMetaAddress,
  NVM_RecordMetaInfo_t* srcMetaInfo,
  uint16_t srcTblEntryIdx,
  uint16_t size
);


/******************************************************************************
 * Name: NvGetRecordFullSize
 * Description: Computes the size of the specified table entry that will 
 *              be written on FLASH memory
 * Parameter(s): [IN] tableEntryIndex - table entry index               
 * Return: the computed size
 *****************************************************************************/
static uint32_t NvGetRecordFullSize
(    
  NvTableEntryId_t tableEntryIndex    
);


/******************************************************************************
 * Name: NvGetTblEntryMetaAddrFromId
 * Description: Gets the table entry meta address based on table entry ID
 * Parameter(s): [IN] searchStartAddress - the search start address
 *               [IN] dataEntryId - table entry ID
 * Return: the value of the meta address
 *****************************************************************************/
#if gNvFragmentation_Enabled_d
static uint32_t NvGetTblEntryMetaAddrFromId
(
  uint32_t searchStartAddress,
  uint16_t dataEntryId
);
#endif /* gNvFragmentation_Enabled_d */


/******************************************************************************
 * Name: NvInternalDefragmentedCopy
 * Description: Performs defragmentation and copy from the source page to 
 *              the destination one
 * Parameter(s): [IN] srcMetaAddr - source page meta address
 *               [IN] pSrcMetaInf - pointer to source page meta information
 *               [IN] srcTblEntryIdx - source page table entry index
 *               [IN] dstMetaAddr - destination meta address
 *               [IN] dstRecordAddr - destination record address (to copy to)
 * Return: the status of the operation
 *****************************************************************************/
#if gNvFragmentation_Enabled_d
static NVM_Status_t NvInternalDefragmentedCopy
(
  uint32_t srcMetaAddr,  
  NVM_RecordMetaInfo_t* pSrcMetaInf,
  uint16_t srcTblEntryIdx,
  uint32_t dstMetaAddr,
  uint32_t dstRecordAddr
);
#endif /* #if gNvFragmentation_Enabled_d */


/******************************************************************************
 * Name: NvCopyPage
 * Description: Copy the active page content to the mirror page. Only the 
 *              latest table entries / elements are copied. A merge operation
 *              is performed before copy if an entry has single elements 
 *              saved priori and newer than the table entry. If one or more
 *              elements were singular saved and the NV page doesn't has a
 *              full table entry saved, then the elements are copied as they
 *              are.  
 * Parameter(s): [IN] skipEntryId - the entry ID to be skipped when page
 *                                  copy is performed
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not 
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be 
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvCopyPage
(    
#if gNvUseExtendedFeatureSet_d
  NvTableEntryId_t skipEntryId
#else
  void
#endif
);


/******************************************************************************
 * Name: NvWritePageCounter
 * Description: Write the page counter value              
 * Parameter(s): [IN] pageId - the ID of the page 
 *               [IN] value - the page counter value that will written to 
 *                            the specified page
 *               
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_Error_c - if the format operation fails
 *****************************************************************************/
static NVM_Status_t NvWritePageCounter
(
  NVM_VirtualPageID_t pageId,
  uint32_t value    
);


/******************************************************************************
 * Name: NvInternalFormat
 * Description: Format the NV storage system. The function erases in place both
 *              virtual pages and then writes the page counter value to first  
 *              virtual page. The provided page counter value is automatically 
 *              incremented and then written to first (active) virtual page.              
 * Parameter(s): [IN] pageCounterValue - the page counter value that will
 *                                       be incremented and then written to
 *                                       active page
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *****************************************************************************/
static NVM_Status_t NvInternalFormat
(
  uint32_t pageCounterValue
);


#if gNvUseExtendedFeatureSet_d
/******************************************************************************
 * Name: NvSaveRamTable
 * Description: Saves the NV table
 * Parameter(s): [IN] pageId - the virtual page ID where the table will be 
 *                             saved 
 * Return: TRUE if table saved successfully, FALSE otherwise
 ******************************************************************************/
static bool_t NvSaveRamTable
(
    NVM_VirtualPageID_t pageId
);

/******************************************************************************
 * Name: NvGetTableSize
 * Description: Retrieves the size of the NV table
 * Parameter(s): [IN] location - specifies if the size shall be the NV FLASH 
 *                               table size (gFLASHTable_c) or the NV RAM table 
 *                               size (gRAMTable_c) 
 * Return: the NV table size
 ******************************************************************************/
static uint32_t NvGetTableSize
(
  uint8_t location
);


/******************************************************************************
 * Name: NvIsRamTableUpdated
 * Description: Checks if the the NV table from RAM memory has changed since
 *              last system reset (e.g. via an OTA transfer)
 * Parameter(s): -
 * Return: TRUE if the NV RAM table has been changed / FALSE otherwise
 ******************************************************************************/
static bool_t NvIsRamTableUpdated
(
  void
);

/******************************************************************************
 * Name: NvGetTableEntry
 * Description: get the NV table entry information stored on FLASH memory
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pDataEntry - a pointer to a memory location where the 
 *                                  entry information will be stored
 * Return: TRUE if the has been found / FALSE otherwise
 ******************************************************************************/
static bool_t NvGetTableEntry
(
  uint16_t tblEntryId,
  NVM_DataEntry_t* pDataEntry
);


/******************************************************************************
 * Name: NvIsNVMFlashAddress
 * Description: check if the address is in Flash
 * Parameter(s): [IN] address
 *               
 * Return: TRUE if the table entry is in Flash / FALSE otherwise
 ******************************************************************************/
#if gUnmirroredFeatureSet_d
static bool_t NvIsNVMFlashAddress
(
  void* address
);
#endif

#endif /* gNvUseExtendedFeatureSet_d */

#endif /* no FlexNVM */


/******************************************************************************
 * Name: NvGetEntryFromDataPtr
 * Description: get table and element indexes based on a generic pointer address
 * Parameter(s): [IN] pData - a pointer to a NVM RAM table
 *               [OUT] pIndex - a pointer to a memory location where the 
 *                              requested indexed will be stored
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PointerOutOfRange_c - if the provided pointer cannot be founded
 *                                    within the RAM table
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetEntryFromDataPtr
(
  void* pData,
  NVM_TableEntryInfo_t* pIndex
);


/******************************************************************************
 * Name: NvWriteRecord
 * Description: writes a record
 * Parameter(s): [IN] tblIndexes - a pointer to table and element indexes
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be 
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvWriteRecord
(        
  NVM_TableEntryInfo_t* tblIndexes
);


/******************************************************************************
 * Name: NvRestoreData
 * Description: restore an element from NVM storage to its original RAM location
 * Parameter(s): [IN] tblIdx - pointer to table and element indexes
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if page is empty
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvRestoreData
(        
  NVM_TableEntryInfo_t* tblIdx
);


/******************************************************************************
 * Name: NvGetTableEntryIndex
 * Description: get the table entry index from the provided ID
 * Parameter(s): [IN] entryId - the ID of the table entry
 * Return: table entry index of gNvInvalidTableEntryIndex_c
 *****************************************************************************/
static uint16_t NvGetTableEntryIndexFromId
(
  NvTableEntryId_t entryId 
);


/******************************************************************************
 * Name: NvAddSaveRequestToQueue
 * Description: Add save request to save requests queue; if the request is 
 *              already stored, ignore the current request  
 * Parameter(s): [IN] ptrTblIdx - pointer to table index
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_SaveRequestRejected_c - if the request couldn't be queued         
 ******************************************************************************/
static NVM_Status_t NvAddSaveRequestToQueue
(
  NVM_TableEntryInfo_t* ptrTblIdx
);

/******************************************************************************
 * Name: NvIntervalTimerCallback
 * Description: Callback function of the timer used by the NvSaveOnInterval()
 * Parameter(s): [IN] timerID - timer ID
 * Return: -
 ******************************************************************************/
static void NvIntervalTimerCallback
(
  void*
);

/******************************************************************************
 * Name: GetRandomRange
 * Description: Returns a random number between 'low' and 'high'
 * Parameter(s): [IN] low, high - generated number range
 * Return: 0..255
 ******************************************************************************/
static uint8_t GetRandomRange
(
  uint8_t low, 
  uint8_t high
);

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /*** FlexNVM ***/

/******************************************************************************
 * Name: NvGetFlexLastMetaInfo
 * Description: Get FlexRAM last meta information address 
 * Parameter(s): -
 * Return: the address of the last valid meta information       
 ******************************************************************************/
static uint32_t NvGetFlexLastMetaInfo
(
  void
);

/******************************************************************************
 * Name: NvGetFlexMetaInfoFromId
 * Description: Get FlexRAM meta information tag from table entry ID 
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pMetaInfo - a pointer to a memory location where the
 *                                 meta information tag will be stored
 * Return: -
 ******************************************************************************/
static void NvGetFlexMetaInfoFromId
(
  NvTableEntryId_t tblEntryId,
  NVM_FlexMetaInfo_t* pMetaInfo
);

/******************************************************************************
 * Name: NvCheckNvmTableForFlexRAMUsage
 * Description: Check if the existing NVM table fits within the FlexRAM window 
 * Parameter(s): -
 * Return: gNVM_NvTableExceedFlexRAMSize_c - the table exceed the size of
 *                                           FlexRAM window
 *         gNVM_OK_c - the table fits within the size of window FlexRAM window                                  
 ******************************************************************************/
static NVM_Status_t NvCheckNvmTableForFlexRAMUsage
(
  void
);

#endif /* gNvUseFlexNVM_d */
#endif /* gNvStorageIncluded_d */


/*****************************************************************************
 *****************************************************************************
 * Private memory declarations
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/*
 * Name: mNvActivePageId
 * Description: variable that holds the ID of the active page
 */
static NVM_VirtualPageID_t mNvActivePageId;

/*
 * Name: mNvPageCounter
 * Description: page counter, used to validate the entire virtual page
 *              and also to provide statistical information about
 *              how many times the virtual page was erased
 */
static NV_baseType mNvPageCounter = 0;

/*
 * Name: mNvVirtualPageProperty
 * Description: virtual page properties
 */
static NVM_VirtualPageProperties_t mNvVirtualPageProperty[gNvVirtualPagesCount_c];

/*
 * Name: mNvCopyOperationIsPending
 * Description: a flag that a indicates that a page copy operation is requested
 */
static bool_t mNvCopyOperationIsPending = FALSE;

/*
 * Name: mNvSkipTableEntryId
 * Description: table entry ID to be skipped when a page copy is requested
 */
#if gNvUseExtendedFeatureSet_d
static NvTableEntryId_t mNvSkipTableEntryId;
#endif

/*
 * Name: mNvErasePgCmdStatus
 * Description: a data structure used to erase a virtual page. The erase of a 
 *              virtual page is performed in idle task, in a sector-by-sector
 *              manner. When the idle task runs, if the erase pending flag is 
 *              set, only one flash sector will be erased. Therefore, the
 *              virtual page will be entirely erased after several runs of 
 *              idle task
 */
static NVM_ErasePageCmdStatus_t mNvErasePgCmdStatus;

/*
 * Name: maNvRecordsCpyIdx
 * Description: An array that stores the indexes of the records already copied; 
 *              Used by the defragmentation process.
 */
#if gNvFragmentation_Enabled_d
static uint16_t maNvRecordsCpyIdx[gNvRecordsCopiedBufferSize_c];
#endif /* gNvFragmentation_Enabled_d */

#if gNvUseExtendedFeatureSet_d
/*
 * Name: mNvTableSizeInRAM
 * Description: the size of the NV table stored in RAM memory
 */
static uint32_t mNvTableSizeInRAM;

/*
 * Name: mNvTableSizeInFlash
 * Description: the size of the NV table stored in the FLASH memory
 */
static uint32_t mNvTableSizeInFlash;

/*
 * Name: mNvTableMarker 
 * Description: FLASH NV table marker, used only for code readability 
 *              (when applying the sizeof() operator to it) 
 */
static NV_baseType mNvTableMarker = gNvTableMarker_c;

/*
 * Name: mNvTableUpdated
 * Description: boolean flag used to mark if the NV table from the RAM memory
 *              has been changed. Set (or left untouched) only at module initialization,
 *              when the existing NV FLASH table (if any) is compared against
 *              the NV RAM table.  
 */
static bool_t mNvTableUpdated;

#endif /* gNvUseExtendedFeatureSet_d */

#endif /* no FlexNVM */

/*
 * Name: mNvModuleInitialized
 * Description: variable that holds the NVM initialisation status
 */
static bool_t mNvModuleInitialized = FALSE;

/*
 * Name: mNvCriticalSectionFlag
 * Description: If this counter is != 0, do not save to NV Storage
 */
static uint8_t mNvCriticalSectionFlag = 0;

/*
 * Name: gNvMinimumTicksBetweenSaves
 * Description: Minimum number of calls to NvTimerTick() between saves of a given data set 
 */
static NvSaveInterval_t gNvMinimumTicksBetweenSaves = gNvMinimumTicksBetweenSaves_c;

/*
 * Name: gNvCountsBetweenSaves
 * Description: Minimum number of calls to NvSaveOnIdle() between saves of a given data set
 */
static NvSaveCounter_t gNvCountsBetweenSaves = gNvCountsBetweenSaves_c;

/*
 * Name: mNvPendingSavesQueue
 * Description: a queue used for storing information about the pending saves
 */
static NVM_SaveQueue_t mNvPendingSavesQueue;

/*
 * Name: maDatasetInfo
 * Description: Data set info table
 */
static NVM_DatasetInfo_t maDatasetInfo[gNvTableEntriesCountMax_c];

/*
 * Name: mNvSaveOnIntervalEvent
 * Description: flag used to signal an 'SaveOnInterval' event
 */
static bool_t mNvSaveOnIntervalEvent;

/*
 * Name: mNvSaveOnIntervalTimerID
 * Description: the ID of timer used by the Save-On-Interval functionality
 */
static tmrTimerID_t mNvSaveOnIntervalTimerID;
/*
 * Name: mNVMMutexId
 * Description: mutex used to ensure NVM functions thread switch safety 
 */
mutex_t mNVMMutex;
#endif /* gNvStorageIncluded_d */

/*****************************************************************************
 *****************************************************************************
 * Public memory declarations
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/*
 * Name: NV_STORAGE_END_ADDRESS
 * Description: NV_STORAGE_END_ADDRESS from linker command file is used by this code
 *              as Raw Sector Start Address. This should not be misleading because 
 *              ENVM module writes meta information in address-ascending order 
 *              and records in address-descending order.
 */
extern uint32_t NV_STORAGE_END_ADDRESS[];

/*
 * Name: NV_STORAGE_SECTOR_SIZE
 * Description: external symbol from linker command file, it represents the size
 *              of a FLASH sector 
 */
extern uint32_t NV_STORAGE_SECTOR_SIZE[];

/*
 * Name:  NV_STORAGE_MAX_SECTORS
 * Description: external symbol from linker command file, it represents the sectors
 *              count used by the ENVM storage system; it has to be a multiple of 2 
 */
extern uint32_t  NV_STORAGE_MAX_SECTORS[];

#endif /* no FlexNVM */


#endif /* gNvStorageIncluded_d */


/*****************************************************************************
 *****************************************************************************
 * Private functions
 *****************************************************************************
 *****************************************************************************/

#if gNvStorageIncluded_d
/******************************************************************************
 * Name: __NvRegisterTableEntry
 * Description: The function tries to register a new table entry within an
 *              existing NV table. If the NV table contained an erased (invalid)
 *              entry, the entry will be overwritten with a new one (provided
 *              by the mean of this function arguments)      
 * Parameter(s): [IN] ptrData - generic pointer to RAM data to be registered
 *                              within the NV storage system
 *               [IN] uniqueId - an unique ID of the table entry
 *               [IN] elemCount - how many elements the table entry contains
 *               [IN] elemSize - the size of an element 
 *               [IN] overwrite - if an existing table entry shall be 
 *                                overwritten
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *****************************************************************************/
#if gNvUseExtendedFeatureSet_d
static NVM_Status_t __NvRegisterTableEntry
(
  void* ptrData,
  NvTableEntryId_t uniqueId,
  uint16_t elemCount,
  uint16_t elemSize,
  bool_t overwrite
)
{

  uint16_t loopCnt = 0;
  uint16_t nullPos = gNvTableEntriesCountMax_c;

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  if((gNvInvalidDataEntry_c == uniqueId) || (gNvEndOfTableId_c == uniqueId))
  {
    return gNVM_RegisterFailure_c;
  }

  while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
  {
    if(NULL == pNVM_DataTable[loopCnt].pData)
    {
      nullPos = loopCnt;        
    }

    if(pNVM_DataTable[loopCnt].DataEntryID == uniqueId)            
    {
      if(overwrite)
      {
        pNVM_DataTable[loopCnt].pData= ptrData;                
        pNVM_DataTable[loopCnt].ElementsCount = elemCount;
        pNVM_DataTable[loopCnt].ElementSize = elemSize;
        return gNVM_OK_c;
      }
      else
      {
        return gNVM_AlreadyRegistered;
      }        
    }
    /* increment the loop counter */
    loopCnt++;
  }

  if(gNvTableEntriesCountMax_c != nullPos)
  {
    pNVM_DataTable[nullPos].pData= ptrData;
    pNVM_DataTable[nullPos].DataEntryID = uniqueId;
    pNVM_DataTable[nullPos].ElementsCount = elemCount;
    pNVM_DataTable[nullPos].ElementSize = elemSize;
    return gNVM_OK_c;
  }        

  return gNVM_RegisterFailure_c;
}
#endif /* gNvUseExtendedFeatureSet_d */

/******************************************************************************
 * Name: __NvAtomicSave
 * Description: The function performs an atomic save of the entire NV table
 *              to the storage system. The operation is performed
 *              in place (atomic).
 * Parameter(s):  [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
static NVM_Status_t __NvAtomicSave
(
  bool_t ignoreCriticalSectionFlag
)
{
  NVM_Status_t status = gNVM_OK_c;
  index_t loopCnt = 0;

  while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
  {
    status = NvSyncSave(pNVM_DataTable[loopCnt].pData, TRUE, ignoreCriticalSectionFlag);

    if((gNVM_CriticalSectionActive_c == status) || (gNVM_NullPointer_c == status))
    {
      /* skip */
      loopCnt++;
      continue;
    }

    if(gNVM_OK_c != status)
    {
      /* error */
      break;
    }

    /* increment the loop counter */
    loopCnt++;
  }
  return status;
}
/******************************************************************************
 * Name: __NvSyncSave
 * Description: The function saves the pointed element or the entire table
 *              entry to the storage system. The save operation is not
 *              performed on the idle task but within this function call.
 * Parameter(s): [IN] ptrData - a pointer to data to be saved
 *               [IN] saveAll - specifies if the entire table entry shall be 
 *                              saved or only the pointed element
 *               [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section                             
 *****************************************************************************/
static NVM_Status_t __NvSyncSave
(
  void* ptrData,
  bool_t saveAll,
  bool_t ignoreCriticalSectionFlag
)
{
  NVM_TableEntryInfo_t tblIdx;
  NVM_Status_t status;

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  if(mNvCriticalSectionFlag && !ignoreCriticalSectionFlag)
  {
    NvAddSaveRequestToQueue(&tblIdx);
    return gNVM_CriticalSectionActive_c;
  }

  if(NULL == ptrData)
  {
    return gNVM_NullPointer_c;
  }

  if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
  {
    return status;
  }

  tblIdx.saveRestoreAll = saveAll;

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
  if((status = NvWriteRecord(&tblIdx)) == gNVM_PageCopyPending_c)
  {
    /* copy active page */
#if gNvUseExtendedFeatureSet_d            
  #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
    if((status = NvCopyPage(gNvCopyAll_c)) != gNVM_OK_c)
    {
      FSCI_MsgNVVirtualPageMonitoring(FALSE,status);
      return status;
    }
    FSCI_MsgNVVirtualPageMonitoring(FALSE,status);
  #else
    if((status = NvCopyPage(gNvCopyAll_c)) != gNVM_OK_c)
    {
      return status;
    }
  #endif
#else    
    if((status = NvCopyPage()) != gNVM_OK_c)
    {
      return status;
    }
#endif /* #if gNvUseExtendedFeatureSet_d */
    
    mNvCopyOperationIsPending = FALSE;

    /* erase old page */
    NV_FlashEraseSector(&gFlashConfig, 
                        mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvRawSectorStartAddress,                                               
                        mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvTotalPageSize,
                        gFlashLaunchCommand);

    /* blank check */
    if(FTFx_OK == FlashVerifySection(&gFlashConfig, mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvRawSectorStartAddress,
        mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvTotalPageSize / FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT, READ_NORMAL_MARGIN, gFlashLaunchCommand))
    {
      mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
      mNvErasePgCmdStatus.NvErasePending = FALSE;
    }

    /* write record */
    status = NvWriteRecord(&tblIdx);
  }
#else /* FlexNVM */
  /* write record */
  status = NvWriteRecord(&tblIdx);
  /* wait for EEPROM system to be ready (fix@ENGR283453) */
  while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET,  FTFx_SSD_FCNFG_EEERDY)));
#endif

  return status;
}


/******************************************************************************
 * Name: __NvEraseEntryFromStorage
 * Description: The function removes a table entry within the existing NV 
 *              table. 
 * Parameter(s): [IN] ptrData - a pointer to an existing RAM data that is
 *                              managed by the NV storage system    
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *****************************************************************************/
#if gNvUseExtendedFeatureSet_d
static NVM_Status_t __NvEraseEntryFromStorage
(
  void* ptrData
)
{
  NVM_TableEntryInfo_t tblIdx;    
  uint16_t tableEntryIndex;

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */    
  index_t loopCnt;    
  NVM_Status_t status;
#endif

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  if(mNvCriticalSectionFlag)
  {
    return gNVM_CriticalSectionActive_c;
  }

  if(NULL == ptrData)
  {
    return gNVM_NullPointer_c;
  }

  if(gNVM_PointerOutOfRange_c == NvGetEntryFromDataPtr(ptrData, &tblIdx))
  {
    return gNVM_PointerOutOfRange_c;
  }

  if(gNvInvalidDataEntry_c == tblIdx.entryId)
  {
    /* element already deleted from RAM table, it can be assumed that also the associated NVM records are erased */
    return gNVM_OK_c;
  }

  if((tableEntryIndex = NvGetTableEntryIndexFromId(tblIdx.entryId)) == gNvInvalidTableEntryIndex_c)
  {
    return gNVM_InvalidTableEntry_c;
  }

  /* invalidate the table entry */
  pNVM_DataTable[tableEntryIndex].DataEntryID = gNvInvalidDataEntry_c;

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

  /* erase the table entry by making a copy of the active page to the inactive one, 
   * but skipping while copying the table entry to be erased */     
  mNvSkipTableEntryId = tblIdx.entryId;
  
  /* make a request to make a page copy */
  mNvCopyOperationIsPending = TRUE;

  return gNVM_OK_c;
#else /* FlexNVM */

  /* format the FlexRAM window */
  NvFormat();

  /* re-write the entire NVM table */    
  loopCnt = 0;
  while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
  {
    NvGetEntryFromDataPtr(pNVM_DataTable[loopCnt].pData, &tblIdx);

    if(gNvInvalidDataEntry_c == tblIdx.entryId)
    {
      loopCnt++;
      continue;
    }

    if(gNVM_OK_c != (status = NvWriteRecord(&tblIdx)))
    {
      return status;
    }
    /* increment the loop counter */
    loopCnt++;
  }
  return status;
#endif /* FlexNVM */
}
#endif /* gNvUseExtendedFeatureSet_d */

/******************************************************************************
 * Name: __NvFormat
 * Description: Format the NV storage system. The function erases both virtual
 *              pages and then writes the page counter to active page.              
 * Parameter(s): -
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_CriticalSectionActive_c - if the system has entered in a 
 *                                        critical section
 *****************************************************************************/
static NVM_Status_t __NvFormat
(
  void
)
{

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */  

  uint32_t pageCounterValue;  
  NVM_Status_t status;

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  if(mNvCriticalSectionFlag)
  {
    return gNVM_CriticalSectionActive_c;
  }

  NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress, (uint8_t*)&pageCounterValue, sizeof(pageCounterValue));        

  if((status = NvInternalFormat(pageCounterValue)) == gNVM_OK_c)
  {     
    /* update last meta info address */
    (void)NvGetLastMetaInfoAddress();
  }
  #if gUnmirroredFeatureSet_d
  {
    uint16_t loopCnt;
    for(loopCnt = 0; loopCnt < (index_t)gNvTableEntriesCountMax_c; loopCnt++)
    {
      if((NULL == pNVM_DataTable[loopCnt].pData) && (gNvEndOfTableId_c == pNVM_DataTable[loopCnt].DataEntryID))
      {
        break;
      }
      if(pNVM_DataTable[loopCnt].DataEntryType == gNVM_NotMirroredInRam_c)
      {
        uint8_t indexEnt;
        for(indexEnt = 0; indexEnt < (index_t)pNVM_DataTable[loopCnt].ElementsCount; indexEnt++)
        {
          ((uint8_t**)pNVM_DataTable[loopCnt].pData)[indexEnt] = NULL;
        }
      } 
    }
  }   
  #endif
  return status;

#else /* FlexNVM */

  uint8_t buff[gNvFlexFormatBufferSize_c];
  uint32_t addr;
  uint16_t size;

  FLib_MemSet(buff, gNvErasedFlashCellValue_c, gNvFlexFormatBufferSize_c);

  addr = gFlashConfig.EERAMBase;
  size = gFlashConfig.EEESize;

  while(size)
  {
    /* wait for EEPROM system to be ready */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET , FTFx_SSD_FCNFG_EEERDY)));

    if(FTFx_OK != EEEWrite(&gFlashConfig, addr, sizeof(buff), buff))        
    {
      return gNVM_FormatFailure_c;
    }
    size -= gNvFlexFormatBufferSize_c;
    addr += gNvFlexFormatBufferSize_c;
  }    
  return gNVM_OK_c;
#endif /* gNvUseFlexNVM_d */

}

/******************************************************************************
 * Name: __NvIdle
 * Description: Called from the idle task (bare-metal) or NVM_Task (MQX,
 *              FreeRTOS) to process the pending saves, erase or copy 
 *              operations.
 * Parameters: -
 * Return: -
 ******************************************************************************/
static void __NvIdle
(
  void
)
{

  NVM_TableEntryInfo_t tblIdx;
  uint16_t tableEntryIdx;
  uint8_t timerJitter;

  if (!mNvModuleInitialized || mNvCriticalSectionFlag) 
  {
    return;
  }

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */    

  if(mNvCopyOperationIsPending)
  {
#if gNvUseExtendedFeatureSet_d    
  #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVVirtualPageMonitoring(TRUE,gNVM_OK_c);
    FSCI_MsgNVVirtualPageMonitoring(FALSE,NvCopyPage(mNvSkipTableEntryId));
  #else
    (void)NvCopyPage(mNvSkipTableEntryId);
  #endif
#else
    (void)NvCopyPage();
#endif /* #if gNvUseExtendedFeatureSet_d */
    mNvCopyOperationIsPending = FALSE;
  }    

  if(mNvErasePgCmdStatus.NvErasePending)
  {
    uint32_t status;

    if(mNvErasePgCmdStatus.NvSectorAddress >= mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvRawSectorEndAddress)
    {
      /* all sectors of the page had been erased */
      mNvVirtualPageProperty[mNvErasePgCmdStatus.NvPageToErase].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
      mNvErasePgCmdStatus.NvErasePending = FALSE;
      return;
    }

    /* erase */
    status = NV_FlashEraseSector(&gFlashConfig, 
                                 mNvErasePgCmdStatus.NvSectorAddress, 
                                 (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE),
                                 gFlashLaunchCommand);
#if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVPageEraseMonitoring(mNvErasePgCmdStatus.NvSectorAddress, status);
#else
    (void)status;
#endif

    /* blank check */
    if(FTFx_OK == FlashVerifySection(&gFlashConfig, mNvErasePgCmdStatus.NvSectorAddress,
        (uint32_t)(NV_STORAGE_SECTOR_SIZE) / FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT, READ_NORMAL_MARGIN, gFlashLaunchCommand))
    {
      mNvErasePgCmdStatus.NvSectorAddress += (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE);
      return;
    }        
  }
#endif

  /* process the save-on-interval requests */    
  if(mNvSaveOnIntervalEvent)
  {  
    if(__NvTimerTick(mNvSaveOnIntervalEvent)) 
    {
      if(!TMR_IsTimerActive(mNvSaveOnIntervalTimerID))
      {
        timerJitter = GetRandomRange(0,255);
        TMR_StartSingleShotTimer(mNvSaveOnIntervalTimerID,
                                 TmrSeconds(1) + timerJitter - 128,
                                 NvIntervalTimerCallback,
                                 NULL);
      }
    }
    mNvSaveOnIntervalEvent = FALSE;
  }
  
  /* process the save-on-idle requests */
  if(NvGetPendingSavesCount(&mNvPendingSavesQueue))
  {
    while(NvPopPendingSave(&mNvPendingSavesQueue, &tblIdx))
    {            
      tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

      if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
      {
        continue;
      }            

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */            
      if(NvWriteRecord(&tblIdx) == gNVM_PageCopyPending_c)
      {
        NvAddSaveRequestToQueue(&tblIdx);
        break;
      }
#else /* FlexNVM */
      NvWriteRecord(&tblIdx);
#endif

      if(tblIdx.saveRestoreAll)
      {
        maDatasetInfo[tableEntryIdx].saveNextInterval = FALSE;
        maDatasetInfo[tableEntryIdx].countsToNextSave = gNvCountsBetweenSaves_c;
      }
    }
  }

}

/******************************************************************************
 * Name: __NvRestoreDataSet
 * Description: copy the most recent version of the element/table entry pointed 
 *              by ptrData from NVM storage system to RAM memory
 * Parameter(s): [IN] ptrData - pointer to data (element) to be restored 
 *               [IN] restoreAll - if FALSE restores a single element
 *                               - if TRUE restores an entire table entry
 * Return: status of the restore operation
 *****************************************************************************/
static NVM_Status_t __NvRestoreDataSet
(
  void* ptrData,    
  bool_t restoreAll
)
{
  NVM_TableEntryInfo_t tblIdx;

  if(!mNvModuleInitialized)
  {
#if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVRestoreMonitoring(0, TRUE, (uint8_t)gNVM_ModuleNotInitialized_c);
#endif
    return gNVM_ModuleNotInitialized_c;
  }

  if(NULL == ptrData)
  {
#if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVRestoreMonitoring(0, TRUE, (uint8_t)gNVM_NullPointer_c);
#endif
    return gNVM_NullPointer_c;
  }

#if gNvFragmentation_Enabled_d
  tblIdx.saveRestoreAll = restoreAll;
#else
  tblIdx.saveRestoreAll = TRUE;
#endif /* gNvFragmentation_Enabled_d */

  if(NvGetEntryFromDataPtr(ptrData, &tblIdx) != gNVM_OK_c)
  {
#if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVRestoreMonitoring(tblIdx.entryId, TRUE, (uint8_t)gNVM_NullPointer_c);  
#endif
    return gNVM_PointerOutOfRange_c;
  }

#if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
  {
    NVM_Status_t nvmStatus;
    FSCI_MsgNVRestoreMonitoring(tblIdx.entryId, TRUE, (uint8_t)gNVM_OK_c);
    nvmStatus=NvRestoreData(&tblIdx);
    FSCI_MsgNVRestoreMonitoring(tblIdx.entryId, FALSE, (uint8_t)nvmStatus);
    return nvmStatus;
  }
#else
  return NvRestoreData(&tblIdx);    
#endif    
}

/******************************************************************************
 * Name: __NvTimerTick
 * Description: Called from the idle task to process save-on-interval requests
 * Parameters: [IN] countTick - enable/disable tick count
 * Return: FALSE if the timer tick counters for all data sets have reached 
 *         zero. In this case, the timer can be turned off.
 *         TRUE if any of the data sets' timer tick counters have not yet
 *         counted down to zero. In this case, the timer should be active
 ******************************************************************************/
static bool_t __NvTimerTick
(
  bool_t countTick
)
{

  bool_t fTicksLeft;                
  NVM_TableEntryInfo_t tblIdx;
  uint16_t idx;

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  idx = 0;
  fTicksLeft = FALSE;

  if(countTick) 
  {
    while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
    {
      if(maDatasetInfo[idx].ticksToNextSave)
      {
        --maDatasetInfo[idx].ticksToNextSave;
      }

      if(maDatasetInfo[idx].ticksToNextSave)
      {
        fTicksLeft = TRUE;
      }

      if(maDatasetInfo[idx].saveNextInterval && !maDatasetInfo[idx].ticksToNextSave)
      {
        if(!mNvCriticalSectionFlag)
        {
          tblIdx.entryId = pNVM_DataTable[idx].DataEntryID;
          tblIdx.elementIndex = 0;
          tblIdx.saveRestoreAll = TRUE;
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
          if(NvWriteRecord(&tblIdx) == gNVM_PageCopyPending_c)
          {
            NvAddSaveRequestToQueue(&tblIdx);
          }                                                   
#else /* FlexNVM */
          NvWriteRecord(&tblIdx);
#endif
          maDatasetInfo[idx].saveNextInterval = FALSE;
          maDatasetInfo[idx].countsToNextSave = gNvCountsBetweenSaves_c;
        }
        else
        {                    
          tblIdx.entryId = pNVM_DataTable[idx].DataEntryID;
          tblIdx.elementIndex = 0;
          tblIdx.saveRestoreAll = TRUE;
          /* push the pending save to pending queue */
          NvAddSaveRequestToQueue(&tblIdx);
        }
      }        
      /* increment the loop counter */
      idx++;
    }
  }
  return fTicksLeft;
}                                       /* NvTimerTick() */


/******************************************************************************
 * Name: __NvSaveOnCount
 * Description: Decrement the counter. Once it reaches 0, the next call to 
 *              NvIdle() will save the entire table entry (all elements).
 * Parameters: [IN] ptrData - pointer to data to be saved
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function 
 ******************************************************************************/
static NVM_Status_t __NvSaveOnCount
(
  void* ptrData
)
{

  NVM_Status_t status;
  NVM_TableEntryInfo_t tblIdx;
  uint16_t tableEntryIdx;

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  if(NULL == ptrData)
  {
    return gNVM_NullPointer_c;
  }

  /* get the NVM table entry */
  if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
  {
    return status;
  }

  if(gNvInvalidDataEntry_c == tblIdx.entryId)
  {
    return gNVM_InvalidTableEntry_c;
  }

  tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

  if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
  {
    return gNVM_InvalidTableEntry_c;
  }

  if(maDatasetInfo[tableEntryIdx].countsToNextSave)
  {
    --maDatasetInfo[tableEntryIdx].countsToNextSave;
  }
  else
  {
    /* all the elements of the NVM table entry will be saved */
    tblIdx.saveRestoreAll = TRUE;

    status = NvAddSaveRequestToQueue(&tblIdx);        
  }

  return status;
}                                       /* NvSaveOnCount() */


/******************************************************************************
 * Name: __NvSaveOnInterval
 * Description:  save no more often than a given time interval. If it has 
 *               been at least that long since the last save,
 *               this function will cause a save the next time the idle 
 *               task runs.
 * Parameters: [IN] ptrData - pointer to data to be saved
 * NOTE: this function saves all the element of the table entry pointed by
 *       ptrData 
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function 
 ******************************************************************************/
static NVM_Status_t __NvSaveOnInterval
(
  void* ptrData
)
{

  NVM_Status_t status;
  NVM_TableEntryInfo_t tblIdx;
  uint16_t tableEntryIdx;

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  if(NULL == ptrData)
  {
    return gNVM_NullPointer_c;
  }

  if(gTmrInvalidTimerID_c == mNvSaveOnIntervalTimerID)
  {
    /* try to allocate the timer used by the save-on-interval functionality */      
    mNvSaveOnIntervalTimerID = TMR_AllocateTimer();

    if(gTmrInvalidTimerID_c == mNvSaveOnIntervalTimerID)
    {
      return gNVM_InvalidTimerID_c;
    }
  }

  /* get the NVM table entry */
  if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
  {
    return status;
  }

  if(gNvInvalidDataEntry_c == tblIdx.entryId)
  {
    return gNVM_InvalidTableEntry_c;
  }

  tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

  if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
  {
    return gNVM_InvalidTableEntry_c;
  }

  if(maDatasetInfo[tableEntryIdx].saveNextInterval == FALSE)
  {
    maDatasetInfo[tableEntryIdx].ticksToNextSave = gNvMinimumTicksBetweenSaves;
    maDatasetInfo[tableEntryIdx].saveNextInterval = TRUE;
    mNvSaveOnIntervalEvent = TRUE;
  }

  return status;
}
/******************************************************************************
 * Name: __NvSaveOnIdle
 * Description: Save the data pointed by ptrData on the next call to NvIdle()
 * Parameter(s): [IN] ptrData - pointer to data to be saved
 *               [IN] saveRestoreAll - specify if all the elements from the NVM table
 *                              entry shall be saved
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_Error_c - in case of error(s)
 *         Note: see also return codes of NvGetEntryFromDataPtr() function         
 ******************************************************************************/
static NVM_Status_t __NvSaveOnIdle
(
  void* ptrData,
  bool_t saveAll
)
{    
  NVM_Status_t status;
  NVM_TableEntryInfo_t tblIdx;

  if(!mNvModuleInitialized)
  {
    return gNVM_ModuleNotInitialized_c;
  }

  if(NULL == ptrData)
  {
    return gNVM_NullPointer_c;
  }

  /* get the NVM table entry */
  if((status = NvGetEntryFromDataPtr(ptrData, &tblIdx)) != gNVM_OK_c)
  {
    return status;
  }

  if(gNvInvalidDataEntry_c == tblIdx.entryId)
  {
    return gNVM_InvalidTableEntry_c;
  }

  /* write the save all flag */
#if gNvFragmentation_Enabled_d    
  tblIdx.saveRestoreAll = saveAll;
#else
  tblIdx.saveRestoreAll = TRUE;
#endif /* gNvFragmentation_Enabled_d */

  return NvAddSaveRequestToQueue(&tblIdx);
}
/******************************************************************************
 * Name: __NvModuleInit
 * Description: Initialize the NV storage module
 * Parameter(s): -
 * Return: gNVM_ModuleAlreadyInitialized_c - if the module is already 
 *                                           initialized
 *         gNVM_InvalidSectorsCount_c - if the sector count configured in the
 *                                      project linker file is invalid
 *         gNVM_MetaNotFound_c - if no meta information was found                                       
 *         gNVM_OK_c - module was successfully initialized
 *****************************************************************************/
static NVM_Status_t __NvModuleInit
(
  void
)
{

  index_t loopCnt;
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */    
  #if gNvUseExtendedFeatureSet_d
  uint32_t pageCounterValue;
#endif /* gNvUseExtendedFeatureSet_d */
#endif    
  bool_t  eot = FALSE; /* end of table marker flag */

  if(mNvModuleInitialized)
  {
    return gNVM_ModuleAlreadyInitialized_c;
  }
  /* Initialize flash HAL driver */
  NV_Init();
  
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */    
  /* check linker file symbol definition for sector count; it should be multiple of 2 */
  if((uint32_t)((uint8_t*) NV_STORAGE_MAX_SECTORS) >> 1 != (uint32_t)((uint8_t*) NV_STORAGE_MAX_SECTORS) - 
      ((uint32_t)((uint8_t*) NV_STORAGE_MAX_SECTORS) >> 1))
  {
    return gNVM_InvalidSectorsCount_c;
  }
#endif

/* check the RAM table to have 'End-Of-Table' terminator */
  for(loopCnt = 0; loopCnt < (index_t)gNvTableEntriesCountMax_c; loopCnt++)
  {
    if((NULL == pNVM_DataTable[loopCnt].pData) && (gNvEndOfTableId_c == pNVM_DataTable[loopCnt].DataEntryID))
    {
      eot = TRUE;
      break;
    }
  }    
  if(!eot)
  {
    return gNVM_MissingEndOfTableMarker_c;
  }

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */    
  /* check if the NVM table fits within the size of the FlexRAM window */
  if(gNVM_OK_c != NvCheckNvmTableForFlexRAMUsage())
  {
    return gNVM_NvTableExceedFlexRAMSize_c;
  }    
#endif /* FlexNVM */

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */    
  /* avoid compiler warnings, only sizeof(mNvTableMarker) is required in this code */
#if gNvUseExtendedFeatureSet_d
  (void)mNvTableMarker;
#endif /* gNvUseExtendedFeatureSet_d */
#endif

#if (gNvUseFlexNVM_d) && (DEBLOCK_SIZE != 0)  /* FlexNVM */

  /* check data flash IFR map */
  if(gFlashConfig.EEESize == 0)
  {
    return gNVM_NvWrongFlashDataIFRMap_c;
  }

  /* Enable the EERAM */
  SetEEEEnable(&gFlashConfig, EEE_ENABLE, FlashCommandSequence);

#else /* no FlexNVM */   

  /* Initialize the active page ID */
  mNvActivePageId = gVirtualPageNone_c;

  /* First virtual page initialisation */
  mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress = (uint32_t)((uint8_t*)NV_STORAGE_END_ADDRESS);
  mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorsCount = (uint32_t)((uint8_t*) NV_STORAGE_MAX_SECTORS) >> 1;
  mNvVirtualPageProperty[gFirstVirtualPage_c].NvTotalPageSize = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorsCount * 
      (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE);
  mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorEndAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress + 
      mNvVirtualPageProperty[gFirstVirtualPage_c].NvTotalPageSize - 1;

  /* Second virtual page initialisation */
  mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorEndAddress + 1;
  mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorsCount = (uint32_t)((uint8_t*) NV_STORAGE_MAX_SECTORS) >> 1;
  mNvVirtualPageProperty[gSecondVirtualPage_c].NvTotalPageSize = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorsCount * 
      (uint32_t)((uint8_t*)NV_STORAGE_SECTOR_SIZE);
  mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorEndAddress = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress + 
      mNvVirtualPageProperty[gSecondVirtualPage_c].NvTotalPageSize - 1;

  /* no pending erase operations on system initialisation */
  mNvErasePgCmdStatus.NvErasePending = FALSE;

  /* at init, no table entries are to be skipped when copy */
#if gNvUseExtendedFeatureSet_d  
  mNvSkipTableEntryId = gNvCopyAll_c;
#endif /* #if gNvUseExtendedFeatureSet_d */

  /* Initialize the storage system: get active page and page counter */
  NvInitStorageSystem();

#endif

  /* Initialize the pending saves queue */
  NvInitPendingSavesQueue(&mNvPendingSavesQueue);

  /* Initialize the data set info table */
  for(loopCnt = 0; loopCnt < (index_t)gNvTableEntriesCountMax_c; loopCnt++)
  {      
    maDatasetInfo[loopCnt].saveNextInterval = FALSE;
    maDatasetInfo[loopCnt].countsToNextSave = gNvCountsBetweenSaves;
  }

  /* initialize the event used by save-on-interval functionality */
  mNvSaveOnIntervalEvent = FALSE;

  /* initialize the timer used by the save-on-interval functionality */
  mNvSaveOnIntervalTimerID = gTmrInvalidTimerID_c; 

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */

  /* NVM module is now initialized */
  mNvModuleInitialized = TRUE;
  return gNVM_OK_c;

#else /* no FlexNVM */

#if gNvUseExtendedFeatureSet_d  
  /* get the size of the NV table stored in RAM memory */
  mNvTableSizeInRAM = NvGetTableSize(gRAMTable_c);
  /* get the size of the NV table stored in FLASH memory */
  mNvTableSizeInFlash = NvGetTableSize(gFLASHTable_c);

  if(0 == mNvTableSizeInFlash) /* no NV table found in FLASH, format the system */
  {
    NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress,
        (uint8_t*)&pageCounterValue, sizeof(pageCounterValue));
    NvInternalFormat(pageCounterValue); /* will also save the NV table to FLASH memory */
  }
  else /* found an valid NV table in FLASH memory */
  {
    /* check if the RAM table was updated (e.g. new binary image via OTA) */
    mNvTableUpdated = NvIsRamTableUpdated();
    if( mNvTableUpdated )
    {
      if(FTFx_OK == NvGetLastMetaInfoAddress())
      {       
        /* copy the new RAM table and the page content */
        (void)NvCopyPage(gNvCopyAll_c);

        /* NVM module is now initialised */
        mNvModuleInitialized = TRUE;          
        return gNVM_OK_c;
      }

      /* format the system */
      if(gNVM_OK_c == NvInternalFormat(0))
      {     
        if(FTFx_OK == NvGetLastMetaInfoAddress())
        {       
          /* NVM module is now initialised */
          mNvModuleInitialized = TRUE;
          return gNVM_OK_c;
        }
      }
      return gNVM_FormatFailure_c;
    }      
  }
#endif /* gNvUseExtendedFeatureSet_d */  

  /* get the last meta information address */
  if(FTFx_OK == NvGetLastMetaInfoAddress())
  {       
    /* NVM module is now initialized */
    mNvModuleInitialized = TRUE;
    #if gUnmirroredFeatureSet_d
    for(loopCnt = 0; loopCnt < (index_t)gNvTableEntriesCountMax_c; loopCnt++)
    {
      if((NULL == pNVM_DataTable[loopCnt].pData) && (gNvEndOfTableId_c == pNVM_DataTable[loopCnt].DataEntryID))
      {
        break;
      }
      if(pNVM_DataTable[loopCnt].DataEntryType == gNVM_NotMirroredInRam_c)
      {
        uint8_t** pTempDataSet = (uint8_t**)pNVM_DataTable[loopCnt].pData;
        {
          
          (void)__NvRestoreDataSet(pTempDataSet,TRUE);     
        }
      } 
    }   
    #endif
    return gNVM_OK_c;
  }

  /* format the system */
  if(gNVM_OK_c == NvInternalFormat(0))
  {     
    if(FTFx_OK == NvGetLastMetaInfoAddress())
    {       
      /* NVM module is now initialised */
      mNvModuleInitialized = TRUE;
      return gNVM_OK_c;
    }
  }

  return gNVM_FormatFailure_c;

#endif /* no FlexNVM */
}

/******************************************************************************
 * Name: __NvmMoveToRam
 * Description: Move from NVM to Ram an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: pointer to Ram location
 *****************************************************************************/
#if gUnmirroredFeatureSet_d
static NVM_Status_t __NvmMoveToRam
(
  void** ppData
)
{
  NVM_TableEntryInfo_t tblIdx;
  uint16_t tableEntryIndex;
  NVM_Status_t status;
  void* pData=NULL;
    
  /* Check if entry is in flash */
  if(!NvIsNVMFlashAddress(*ppData)&&(*ppData != NULL))
  {
    return gNVM_OK_c;
  }
  
  /* Get entry from NVM table */
  if((status = NvGetEntryFromDataPtr(ppData, &tblIdx)) != gNVM_OK_c)
  {
    return status;
  }
  
  if((tableEntryIndex = NvGetTableEntryIndexFromId(tblIdx.entryId)) == gNvInvalidTableEntryIndex_c)
  {
    return gNVM_InvalidTableEntry_c;
  }
  
  if(gNVM_NotMirroredInRam_c != pNVM_DataTable[tableEntryIndex].DataEntryType)
  {
    return gNVM_IsMirroredDataSet_c;
  }
  
  /* Allocate a buffer for the data set */
  pData = MSG_Alloc(pNVM_DataTable[tableEntryIndex].ElementSize);
  if(pData == NULL)
  {
    return gNVM_NoMemory_c;
  }
  
  /* Write from Flash to Ram */
  if(*ppData != NULL)
  {
    FLib_MemCpy(pData, *ppData, pNVM_DataTable[tableEntryIndex].ElementSize);
  }                     
  
  OSA_EnterCritical(kCriticalDisableInt);
  *ppData = pData;
  OSA_ExitCritical(kCriticalDisableInt);
  /* Check if the address is in ram */
  return gNVM_OK_c;
}
#endif

/******************************************************************************
 * Name: __NvmErase
 * Description: Erase from NVM an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be erased
 * Return: pointer to Ram location
 *****************************************************************************/
#if gUnmirroredFeatureSet_d
static NVM_Status_t __NvmErase
(
  void** ppData
)
{
  NVM_Status_t status;
  NVM_TableEntryInfo_t tblIdx;
  uint16_t tableEntryIndex;
  
  /* Get entry from NVM table */
  if((status = NvGetEntryFromDataPtr(ppData, &tblIdx)) != gNVM_OK_c)
  {
    return status;
  }
  
  if((tableEntryIndex = NvGetTableEntryIndexFromId(tblIdx.entryId)) == gNvInvalidTableEntryIndex_c)
  {
    return gNVM_InvalidTableEntry_c;
  }
  
  if(gNVM_NotMirroredInRam_c != pNVM_DataTable[tableEntryIndex].DataEntryType)
  {
    return gNVM_IsMirroredDataSet_c;
  }
  
  if(!NvIsNVMFlashAddress(*ppData))
  {
    if(*ppData != NULL)
    {
      MSG_Free(*ppData);
    }
    *ppData = NULL;
    return gNVM_OK_c;
  }

  OSA_EnterCritical(kCriticalDisableInt);
  *ppData = NULL;
  OSA_ExitCritical(kCriticalDisableInt);
  return __NvSyncSave(ppData,FALSE,TRUE);
}
#endif
/******************************************************************************
 * Name: NvInitPendingSavesQueue
 * Description: Initialize the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 * Return: TRUE if the pointer is valid, FALSE otherwise
 ******************************************************************************/
static bool_t NvInitPendingSavesQueue
(
  NVM_SaveQueue_t *pQueue
)
{
  if(NULL == pQueue)
  {
    return FALSE;
  }

  pQueue->Head = 0;
  pQueue->Tail = 0;
  pQueue->EntriesCount = 0;

  return TRUE;
}


/******************************************************************************
 * Name: NvPushPendingSave
 * Description: Add a new pending save to the queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [IN] data - data to be saved
 * Return: TRUE if the push operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPushPendingSave
(
  NVM_SaveQueue_t *pQueue, 
  NVM_TableEntryInfo_t data    
)
{
  if(NULL == pQueue)
  {
    return FALSE;
  }

  if((pQueue->Tail == pQueue->Head) && (pQueue->EntriesCount > 0))
  {
#if gFifoOverwriteEnabled_c        
    /* increment the head (read index) */
    pQueue->Head = (pQueue->Head + 1) & ((unsigned char) (gNvPendigSavesQueueSize_c - 1));
#else
    return FALSE;
#endif
  }

  /* Add the item to queue */
  pQueue->QData[pQueue->Tail] = data;

  /* Reset the tail when it reach gNvPendigSavesQueueSize_c */
  pQueue->Tail = (pQueue->Tail + 1) % ((unsigned char) (gNvPendigSavesQueueSize_c));

  /* Increment the entries count */
  if(pQueue->EntriesCount < (unsigned char) (gNvPendigSavesQueueSize_c))
  {
    pQueue->EntriesCount++;
  }

  return TRUE;
}


/******************************************************************************
 * Name: NvPopPendingSave
 * Description: Retrieves the head element from the pending saves queue
 * Parameters: [IN] pQueue - pointer to queue
 *             [OUT] pData - pointer to the location where data will be placed
 * Return: TRUE if the pop operation succeeded, FALSE otherwise
 ******************************************************************************/
static bool_t NvPopPendingSave
(
  NVM_SaveQueue_t *pQueue, 
  NVM_TableEntryInfo_t *pData
)
{
  if( (NULL == pQueue) || (pQueue->EntriesCount <= 0) || ((NULL == pData)) )
  {
    return FALSE;
  }

  *pData = pQueue->QData[pQueue->Head];

  /* Reset the head when it reach gNvPendigSavesQueueSize_c */
  pQueue->Head = (pQueue->Head + 1) % ((unsigned char) (gNvPendigSavesQueueSize_c ));

  /* Decrement the entries count */
  pQueue->EntriesCount--;

  return TRUE;
}
/******************************************************************************
 * Name: NvGetPendingSavesCount
 * Description: self explanatory
 * Parameters: [IN] pQueue - pointer to queue
 * Return: Number of pending saves
 ******************************************************************************/
static uint8_t NvGetPendingSavesCount
(
  NVM_SaveQueue_t *pQueue 
)
{
  if( NULL == pQueue ) 
  {
    return 0;
  }
  return pQueue->EntriesCount;
}


/*****************************************************************
 * The below functions are compiled only if FlexNVM is NOT used 
 *****************************************************************/

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */

/******************************************************************************
 * Name: NvEraseVirtualPage
 * Description: erase the specified page
 * Parameter(s): [IN] pageID - the ID of the page to be erased
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid
 *         gNVM_SectorEraseFail_c - if the page cannot be erased
 *         gNVM_OK_c - if operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvEraseVirtualPage
(
  NVM_VirtualPageID_t pageID
)
{
  uint32_t status;

  if(pageID > gSecondVirtualPage_c)
    return gNVM_InvalidPageID_c;

  /* erase virtual page */
  status = NV_FlashEraseSector(&gFlashConfig, 
                               mNvVirtualPageProperty[pageID].NvRawSectorStartAddress,
                               mNvVirtualPageProperty[pageID].NvTotalPageSize,
                               gFlashLaunchCommand);
  if(FTFx_OK == status)
  {
    return gNVM_OK_c;
  }

  return gNVM_SectorEraseFail_c;
}


/******************************************************************************
 * Name: NvInitStorageSystem
 * Description: Initialize the storage system, retrieve the active page and
 *              the page counter. Called once by NvModuleInit() function.
 * Parameter(s): - 
 * Return: -
 *****************************************************************************/
static void NvInitStorageSystem
(
  void
)
{
  uint32_t firstPageCounterTopValue;
  uint32_t firstPageCounterBottomValue;
  uint32_t secondPageCounterTopValue;
  uint32_t secondPageCounterBottomValue;

  /* read both pages counter values */
  NV_FlashRead(mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress, (uint8_t*)&firstPageCounterTopValue, 
      sizeof(firstPageCounterTopValue));
  NV_FlashRead(mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorEndAddress - sizeof(firstPageCounterBottomValue) + 1,  
      (uint8_t*)&firstPageCounterBottomValue, sizeof(firstPageCounterBottomValue));
  NV_FlashRead(mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress, (uint8_t*)&secondPageCounterTopValue, 
      sizeof(secondPageCounterTopValue));
  NV_FlashRead(mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorEndAddress - sizeof(secondPageCounterBottomValue) + 1, 
      (uint8_t*)&secondPageCounterBottomValue, sizeof(secondPageCounterBottomValue));

  mNvActivePageId = gVirtualPageNone_c;

  /* get the active page */
  if((firstPageCounterTopValue == firstPageCounterBottomValue) && (gPageCounterMaxValue_c != firstPageCounterTopValue)) /* first page is valid */
  {
    if((secondPageCounterTopValue == secondPageCounterBottomValue) && (gPageCounterMaxValue_c != secondPageCounterTopValue)) /* second page is valid */
    {
      if(firstPageCounterTopValue >= secondPageCounterTopValue)
      {
        /* first page is active */
        mNvPageCounter = firstPageCounterTopValue;
        mNvActivePageId = gFirstVirtualPage_c;
        return;
      }

      /* second page is active */
      mNvPageCounter = secondPageCounterTopValue;
      mNvActivePageId = gSecondVirtualPage_c;
      return;
    }

    if(secondPageCounterTopValue != secondPageCounterBottomValue)
    {                
      /* first page is active */
      mNvPageCounter = firstPageCounterTopValue;
      mNvActivePageId = gFirstVirtualPage_c;
      /* request the erase of the second page */
      mNvErasePgCmdStatus.NvPageToErase = gSecondVirtualPage_c;            
      mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress;
      mNvErasePgCmdStatus.NvErasePending = TRUE;
      return;
    }
    
    /* first page is active */
    mNvPageCounter = firstPageCounterTopValue;
    mNvActivePageId = gFirstVirtualPage_c;
    
    if(gNVM_PageIsNotBlank_c == NvVirtualPageBlankCheck(gSecondVirtualPage_c))
    {            
      /* request the erase of the second page */
      mNvErasePgCmdStatus.NvPageToErase = gSecondVirtualPage_c;            
      mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress;
      mNvErasePgCmdStatus.NvErasePending = TRUE;      
    }         
    
    return;
  }

  if(firstPageCounterTopValue != firstPageCounterBottomValue) /* first page is not valid */
  {
    if((secondPageCounterTopValue == secondPageCounterBottomValue) && (gPageCounterMaxValue_c != secondPageCounterTopValue)) /* second page is valid */
    {            
      /* second page is active */
      mNvPageCounter = secondPageCounterTopValue;
      mNvActivePageId = gSecondVirtualPage_c;
      /* request the erase of the first page */
      mNvErasePgCmdStatus.NvPageToErase = gFirstVirtualPage_c;            
      mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress;
      mNvErasePgCmdStatus.NvErasePending = TRUE;
      return;
    }
    
    /* both pages are not valid, format the NV storage system */
    (void)NvInternalFormat(0);
    return;   
  }
  
  if((secondPageCounterTopValue == secondPageCounterBottomValue) && (gPageCounterMaxValue_c != secondPageCounterTopValue)) /* second page is valid */
  {
    /* second page is active */
    mNvPageCounter = secondPageCounterTopValue;
    mNvActivePageId = gSecondVirtualPage_c;
    
    if(gNVM_PageIsNotBlank_c == NvVirtualPageBlankCheck(gFirstVirtualPage_c))
    {
      /* request the erase of the first page */
      mNvErasePgCmdStatus.NvPageToErase = gFirstVirtualPage_c;            
      mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress;
      mNvErasePgCmdStatus.NvErasePending = TRUE;
    }
    return;
  }
  
  /* both pages are not valid, format the NV storage system */
  (void)NvInternalFormat(0);
}

/******************************************************************************
 * Name: NvVirtualPageBlankCheck
 * Description: checks if the specified page is blank (erased) 
 * Parameter(s): [IN] pageID - the ID of the page to be checked
 * Return: gNVM_InvalidPageID_c - if the page ID is not valid 
 *         gNVM_PageIsNotBlank_c - if the page is not blank
 *         gNVM_OK_c - if the page is blank (erased)
 *****************************************************************************/
static NVM_Status_t NvVirtualPageBlankCheck
(
  NVM_VirtualPageID_t pageID
)
{
  if(pageID > gSecondVirtualPage_c)
    return gNVM_InvalidPageID_c;

  if(FTFx_OK != FlashVerifySection(&gFlashConfig, mNvVirtualPageProperty[pageID].NvRawSectorStartAddress,
      (mNvVirtualPageProperty[pageID].NvTotalPageSize / FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT), READ_NORMAL_MARGIN, gFlashLaunchCommand))
  {
    return gNVM_PageIsNotBlank_c;
  }

  return gNVM_OK_c;    
}


/******************************************************************************
 * Name: NvGetLastMetaInfoAddress
 * Description: retrieve and store (update) the last meta information address 
 * Parameter(s): -
 * Return: gNVM_MetaNotFound_c - if no meta information has been found
 *         gNVM_OK_c - if the meta was found and stored (updated)
 *****************************************************************************/
static NVM_Status_t NvGetLastMetaInfoAddress
(    
  void
)
{       
  uint32_t readAddress;  
  uint8_t metaFirstByte;
  NVM_RecordMetaInfo_t metaValue;

#if gNvUseExtendedFeatureSet_d  
  readAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter) + 
      mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker));
#else
  readAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter);
#endif /* gNvUseExtendedFeatureSet_d */

  while(readAddress < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress)
  {
    NV_FlashRead(readAddress, (uint8_t*)&metaFirstByte, sizeof(metaFirstByte));

    if(gNvErasedFlashCellValue_c == metaFirstByte)
    {
      NV_FlashRead(readAddress, (uint8_t*)&metaValue, sizeof(metaValue));

      if(metaValue.rawValue == 0xFFFFFFFFFFFFFFFF)
      {
        if(readAddress == (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter)
#if gNvUseExtendedFeatureSet_d              
                + mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker))
#endif /* gNvUseExtendedFeatureSet_d */
        ))
        {
          mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
          #if gUnmirroredFeatureSet_d 
            mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = gEmptyPageMetaAddress_c;
          #endif
          return gNVM_OK_c;
        }

        readAddress -= sizeof(NVM_RecordMetaInfo_t);

        while(readAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter)
#if gNvUseExtendedFeatureSet_d
                + mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker))
#endif /* gNvUseExtendedFeatureSet_d */
        ))
        {
          NV_FlashRead(readAddress, (uint8_t*)&metaValue, sizeof(metaValue));

          if((metaValue.fields.NvValidationStartByte == metaValue.fields.NvValidationEndByte) &&
              ((gValidationByteSingleRecord_c == metaValue.fields.NvValidationStartByte) || 
                  (gValidationByteAllRecords_c == metaValue.fields.NvValidationStartByte)))
          {
            mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = readAddress;
            #if gUnmirroredFeatureSet_d
            {
              while(readAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter)
                                    + mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker))))
              {
                if(metaValue.fields.NvmRecordOffset == 0)
                {
                  readAddress -= sizeof(NVM_RecordMetaInfo_t);
                  NV_FlashRead(readAddress, (uint8_t*)&metaValue, sizeof(metaValue));
                }
                else
                {
                  mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = readAddress;
                  break;
                }
              }
            }
            #endif
            return gNVM_OK_c;
          }                    
          readAddress -= sizeof(NVM_RecordMetaInfo_t);
        }
        return gNVM_MetaNotFound_c;
      }
      return gNVM_MetaNotFound_c;
    }
    else
    {
      readAddress += sizeof(NVM_RecordMetaInfo_t);
    }
  }    
  return gNVM_MetaNotFound_c;
}


/******************************************************************************
 * Name: NvGetMetaInfo
 * Description: get meta information based on the meta information address
 * Parameter(s): [IN] pageID - the ID of the page
 *               [IN] metaInfoAddress - meta information address
 *               [OUT] pMetaInfo - a pointer to a memory location where the 
 *                                 requested meta information will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_AddressOutOfRange_c - if the provided address is out of range
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetMetaInfo
(
  NVM_VirtualPageID_t pageId,
  uint32_t metaInfoAddress,
  NVM_RecordMetaInfo_t* pMetaInfo
)
{
  /* check address range */
  if(metaInfoAddress < (mNvVirtualPageProperty[pageId].NvRawSectorStartAddress + sizeof(mNvPageCounter)
#if gNvUseExtendedFeatureSet_d        
          + (mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker)))
#endif /* gNvUseExtendedFeatureSet_d */
          ) || metaInfoAddress > mNvVirtualPageProperty[pageId].NvRawSectorEndAddress)
  {
    return gNVM_AddressOutOfRange_c;
  }

  /* read the meta information tag */
  NV_FlashRead(metaInfoAddress, (uint8_t*)pMetaInfo, sizeof(NVM_RecordMetaInfo_t));

  return gNVM_OK_c;    
}


/******************************************************************************
 * Name: NvGetPageFreeSpace
 * Description: return the page free space, in bytes
 * Parameter(s): [OUT] ptrFreeSpace - a pointer to a memory location where the
 *                                    page free space will be stored
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if the page is empty
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetPageFreeSpace
(    
  uint32_t* ptrFreeSpace
)
{
  NVM_RecordMetaInfo_t metaInfo;
  NVM_Status_t retVal;

  if(gEmptyPageMetaAddress_c == mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress)
  {
#if gNvUseExtendedFeatureSet_d    
    *ptrFreeSpace = mNvVirtualPageProperty[mNvActivePageId].NvTotalPageSize - mNvTableSizeInFlash - 
        (2 * (sizeof(mNvPageCounter) + sizeof(mNvTableMarker)));
#else
    *ptrFreeSpace = mNvVirtualPageProperty[mNvActivePageId].NvTotalPageSize - (2 * sizeof(mNvPageCounter));
#endif /* gNvUseExtendedFeatureSet_d */
  }
  else
  {        
    retVal = NvGetMetaInfo(mNvActivePageId, mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress, &metaInfo);

    if(gNVM_OK_c == retVal)
    {
      #if gUnmirroredFeatureSet_d
      {
        if(metaInfo.fields.NvmRecordOffset == 0)
        {
          NVM_RecordMetaInfo_t metaInfoUndeleted;
          NVM_Status_t ret;
          ret=NvGetMetaInfo(mNvActivePageId, mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress, &metaInfoUndeleted);
          if(gNVM_OK_c != ret)
          {
            *ptrFreeSpace = 0;
            return ret;
          }
          else
          {
            metaInfo.fields.NvmRecordOffset =  metaInfoUndeleted.fields.NvmRecordOffset;
          }
        }
      }
      #endif
      *ptrFreeSpace = (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset) - 
          (mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress + 
              sizeof(NVM_RecordMetaInfo_t));
    }
    else
    {            
      *ptrFreeSpace = 0;
    }
  }
  return retVal;
}


/******************************************************************************
 * Name: NvIsMemoryAreaAvailable
 * Description: checks if the specified memory area is blank (erased)
 * Parameter(s): [IN] address - start address
 *               [IN] len - length to be verified
 * Return: TRUE if the area is available (blank), FALSE otherwise
 *****************************************************************************/
static bool_t NvIsMemoryAreaAvailable
(
  uint32_t address,
  uint32_t len
)
{
  uint8_t readBuffer[PGM_SIZE_BYTE];
  uint8_t loopCnt;

  if(len % (uint8_t)PGM_SIZE_BYTE != 0)
  {
    return FALSE;
  }


  if(address < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress ||
      address > mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress)
  {
    return FALSE;
  }

  if((address + len) > mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress + 1)
  {
    return FALSE;
  }

  while(len)
  {

    NV_FlashRead(address,(uint8_t*)&readBuffer[0], (uint8_t)PGM_SIZE_BYTE);
    loopCnt = (uint8_t)PGM_SIZE_BYTE;
    while(loopCnt)
    {
      if(readBuffer[--loopCnt] != gNvErasedFlashCellValue_c)
      {
        return FALSE;
      }            
    }        
    len-=(uint8_t)PGM_SIZE_BYTE;
    address+=(uint8_t)PGM_SIZE_BYTE;
  }
  return TRUE;    
}


/******************************************************************************
 * Name: NvIsRecordCopied
 * Description: Checks if a record or an entire table entry is already copied. 
 *              Called by page copy function.
 * Parameter(s): [IN] pageId - the ID of the page where to perform the check
 *               [IN] metaInf - a pointer to source page meta information tag
 * Return: TRUE if the element is already copied, FALSE otherwise
 *****************************************************************************/
static bool_t NvIsRecordCopied
(
  NVM_VirtualPageID_t pageId,
  NVM_RecordMetaInfo_t* metaInf
)
{
  uint32_t loopAddress;
  uint8_t dataRead[8];
  bool_t retVal;
  uint16_t tmp;

#if gNvUseExtendedFeatureSet_d
  loopAddress = mNvVirtualPageProperty[pageId].NvRawSectorStartAddress + sizeof(mNvPageCounter) + 
      mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker));
#else
  loopAddress = mNvVirtualPageProperty[pageId].NvRawSectorStartAddress + sizeof(mNvPageCounter);         
#endif /* gNvUseExtendedFeatureSet_d */
  
  retVal = FALSE;

  do
  {
    /* read the meta information tag */
    NV_FlashRead(loopAddress, (uint8_t*)dataRead, sizeof(NVM_RecordMetaInfo_t));

    if(dataRead[0] == gNvErasedFlashCellValue_c)
    {
      /* no meta starts with 0xFF */
      break;
    }

    if(dataRead[0] != dataRead[7])
    {
      /* invalid meta */
      loopAddress += sizeof(NVM_RecordMetaInfo_t);
      continue;
    }

    /* get the table entry ID */
    tmp = (uint16_t)(((uint16_t)dataRead[2]<<8) + dataRead[1]);

    if(metaInf->fields.NvmDataEntryID == tmp)
    {  
      if(metaInf->fields.NvValidationStartByte == gValidationByteSingleRecord_c)
      {
        if(dataRead[0] == gValidationByteSingleRecord_c)
        {
          /* get the element index */
          tmp = (uint16_t)(((uint16_t)dataRead[4]<<8) + dataRead[3]);                  
          if(tmp == metaInf->fields.NvmElementIndex)
          {
            retVal = TRUE;
            break;
          }

          /* skip */
          loopAddress += sizeof(NVM_RecordMetaInfo_t);
          continue;
        }              
        retVal = TRUE;
        break;
      }

      if(metaInf->fields.NvValidationStartByte == gValidationByteAllRecords_c)
      {
        if(dataRead[0] == gValidationByteSingleRecord_c)
        {
          /* skip */
          loopAddress += sizeof(NVM_RecordMetaInfo_t);
          continue;
        }
        retVal = TRUE;
        break;
      }

      /* skip */
      loopAddress += sizeof(NVM_RecordMetaInfo_t);
      continue;
    }      

    loopAddress += sizeof(NVM_RecordMetaInfo_t);

  } while(loopAddress < mNvVirtualPageProperty[pageId].NvRawSectorEndAddress);

  return retVal;
}


/******************************************************************************
 * Name: NvInternalCopy
 * Description: Performs a copy of an record / entire table entry
 * Parameter(s): [IN] dstAddress - destination record address
 *               [IN] dstMetaAddress - destination meta address
 *               [IN] srcMetaInfo - source meta information
 *               [IN] srcTblEntryIdx - source table entry index
 *               [IN] size - bytes to copy
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not 
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be 
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvInternalCopy
(
  uint32_t dstAddress,
  uint32_t dstMetaAddress,
  NVM_RecordMetaInfo_t* srcMetaInfo,
  uint16_t srcTblEntryIdx,
  uint16_t size
)
{
  uint16_t innerOffset;
  uint8_t cacheBuffer[gNvCacheBufferSize_c];
  NVM_RecordMetaInfo_t dstMetaInfo;
  uint16_t diffSize = 0;
  uint16_t diffIdx = 0;
  uint16_t ramSize = 0;
  uint8_t misalignedBytes;
  uint8_t loopIdx;
  uint16_t loopEnd;

  /* Initialize the inner offset*/
  innerOffset = 0;

  /* prepare destination page meta info tag and write if after the record is entirely written.
   * the preparation is made here because the 'dstAddress' may change afterwards
   */        
  
  dstMetaInfo.fields = srcMetaInfo->fields; 
  dstMetaInfo.fields.NvmRecordOffset = dstAddress - mNvVirtualPageProperty[(mNvActivePageId+1)%2].NvRawSectorStartAddress;
  
  ramSize = pNVM_DataTable[srcTblEntryIdx].ElementsCount * pNVM_DataTable[srcTblEntryIdx].ElementSize;

  /* if the bytes to copy are less then RAM table entry space, the supplementary bytes to write on the destination page
   * will be retrieved from RAM table entry. This is the case when the RAM table has been updated and the new
   * entry's elements count is greater then the one existing in the previous RAM table, now stored in the FLASH active page
   * (source page) */
  if(size < ramSize)
  {
    diffSize = ramSize - size;
    diffIdx = size / pNVM_DataTable[srcTblEntryIdx].ElementSize;
  }

  while(size)
  {
    if(size > (uint16_t)gNvCacheBufferSize_c)
    {
      /* copy from FLASH to cache buffer */
      NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + srcMetaInfo->fields.NvmRecordOffset + innerOffset, 
          (uint8_t*)&cacheBuffer[0], (uint16_t)gNvCacheBufferSize_c);                    

      /* write to destination page */
      if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, (uint16_t)gNvCacheBufferSize_c, cacheBuffer, gFlashLaunchCommand))
      {
        /* update the destination record address copy */
        dstAddress += (uint16_t)gNvCacheBufferSize_c;
        /* update the record size */
        size -= (uint16_t)gNvCacheBufferSize_c;
        /* update the inner offset value */
        innerOffset += (uint16_t)gNvCacheBufferSize_c;

        continue;
      }
      return gNVM_RecordWriteError_c;
    }
    else
    {
      /* copy from FLASH to cache buffer */
      NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + srcMetaInfo->fields.NvmRecordOffset + innerOffset, 
          (uint8_t*)&cacheBuffer[0], size);
      /* write to destination page */
      if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, (uint16_t)size, cacheBuffer, gFlashLaunchCommand))
      {
        break;
      }
      return gNVM_RecordWriteError_c;
    }
  }

  if(diffSize)
  {
    /* update the destination record address copy */
    dstAddress += size;
    
    /* check alignment and adjust it if necessary */                            
    misalignedBytes = dstAddress - (dstAddress & (~0x03uL));
    
    /* initialise the inner offset */
    innerOffset = 0;

    /* check if the destination is longword aligned or not */
    if(misalignedBytes)
    {
      /* align to previous 32 bit boundary */
      dstAddress &= ~0x03uL;
                                
      /* compute the loop end */
      if(pNVM_DataTable[srcTblEntryIdx].ElementSize < (4-misalignedBytes))
      {
        loopEnd = pNVM_DataTable[srcTblEntryIdx].ElementSize; 
      }
      else
      {
        loopEnd = 4 - misalignedBytes;
      }
        
      /* read from destination page to cache buffer */
      NV_FlashRead(dstAddress, (uint8_t*)&cacheBuffer[0], 4);
                
      /* update with data from RAM */
      for(loopIdx = 0; loopIdx < loopEnd; loopIdx++)
      {
        cacheBuffer[misalignedBytes] = *((uint8_t*)pNVM_DataTable[srcTblEntryIdx].pData + 
                    (diffIdx * pNVM_DataTable[srcTblEntryIdx].ElementSize) + innerOffset);
        innerOffset++;
        misalignedBytes++;
      }

      /* write to Flash destination page */
      if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, PGM_SIZE_BYTE, cacheBuffer, gFlashLaunchCommand))
      {
        return gNVM_RecordWriteError_c;
      }

      /* align to next 32 bit boundary */       
      dstAddress += PGM_SIZE_BYTE;
    }

    /* write to Flash destination page */
    if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstAddress, diffSize - innerOffset,
                               ((uint8_t*)pNVM_DataTable[srcTblEntryIdx].pData + (diffIdx * pNVM_DataTable[srcTblEntryIdx].ElementSize) + innerOffset), 
                               gFlashLaunchCommand))
    {
      return gNVM_RecordWriteError_c;
    }
  }

  /* write the associated record meta information */                
  if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstMetaAddress, sizeof(NVM_RecordMetaInfo_t), (uint8_t*)(&dstMetaInfo), gFlashLaunchCommand))
  {
    return gNVM_MetaInfoWriteError_c;                    
  }    
  return gNVM_OK_c;
}


/******************************************************************************
 * Name: NvGetRecordFullSize
 * Description: Computes the size of the specified table entry that will 
 *              be written on FLASH memory
 * Parameter(s): [IN] tableEntryIndex - table entry index               
 * Return: the computed size
 *****************************************************************************/
static uint32_t NvGetRecordFullSize
(    
  NvTableEntryId_t tableEntryIndex    
)
{
  uint32_t size;
  uint8_t paddingBytes;

  /* compute the RAM size */
  size = pNVM_DataTable[tableEntryIndex].ElementSize * pNVM_DataTable[tableEntryIndex].ElementsCount;

  /* compute the size that will be actually written on FLASH memory */        
  paddingBytes = size % (uint8_t)PGM_SIZE_BYTE;   

  if(paddingBytes)
  {
    size += (uint8_t)((uint8_t)PGM_SIZE_BYTE-paddingBytes);
  }

  return size;
}


/******************************************************************************
 * Name: NvGetTblEntryMetaAddrFromId
 * Description: Gets the table entry meta address based on table entry ID
 * Parameter(s): [IN] searchStartAddress - the search start address
 *               [IN] dataEntryId - table entry ID
 * Return: the value of the meta address
 *****************************************************************************/
#if gNvFragmentation_Enabled_d
static uint32_t NvGetTblEntryMetaAddrFromId
(
  uint32_t searchStartAddress,
  uint16_t dataEntryId
)
{
  NVM_RecordMetaInfo_t metaInfo;

  while(searchStartAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter))
#if gNvUseExtendedFeatureSet_d
          + (mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker)))
#endif /* gNvUseExtendedFeatureSet_d */
       )
  {
    (void)NvGetMetaInfo(mNvActivePageId, searchStartAddress, &metaInfo);

    if((metaInfo.fields.NvValidationStartByte != gValidationByteAllRecords_c) || 
        (metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte))
    {
      searchStartAddress -= sizeof(NVM_RecordMetaInfo_t);
      continue;
    }

    if(metaInfo.fields.NvmDataEntryID == dataEntryId)
    {
      /* found it */
      return searchStartAddress;
    }

    searchStartAddress -= sizeof(NVM_RecordMetaInfo_t);
  }    
  return 0;
}
#endif /* gNvFragmentation_Enabled_d */

/******************************************************************************
 * Name: NvInternalDefragmentedCopy
 * Description: Performs defragmentation and copy from the source page to 
 *              the destination one
 * Parameter(s): [IN] srcMetaAddr - source page meta address
 *               [IN] pSrcMetaInf - pointer to source page meta information
 *               [IN] srcTblEntryIdx - source page table entry index
 *               [IN] dstMetaAddr - destination meta address
 *               [IN] dstRecordAddr - destination record address (to copy to)
 * Return: the status of the operation
 *****************************************************************************/
#if gNvFragmentation_Enabled_d
static NVM_Status_t NvInternalDefragmentedCopy
(
  uint32_t srcMetaAddr,  
  NVM_RecordMetaInfo_t* pSrcMetaInf,
  uint16_t srcTblEntryIdx,
  uint32_t dstMetaAddr,
  uint32_t dstRecordAddr
)
{ 
  uint32_t metaAddress;
  uint32_t tblEntryMetaAddress;
  NVM_RecordMetaInfo_t dstMetaInfo;
  NVM_RecordMetaInfo_t metaInfo;
  uint32_t destination;
  uint16_t elemSize;
  uint16_t elemSizeCopy;
  uint16_t innerOffset;
  uint8_t misalignedBytes;                
  uint16_t recordsCopiedCurrentIdx;    
  uint16_t recordIdx;
  bool_t recordIsCopied;
  /* copy buffers */
  uint8_t srcBuffer[4];
  uint8_t dstBuffer[4];
  /* loop control variables */
  uint16_t loopIdx;    
  uint8_t loopEnd;    
#if gNvUseExtendedFeatureSet_d  
  NVM_DataEntry_t flashDataEntry;
  bool_t fillFromRAM = FALSE;
#endif /* gNvUseExtendedFeatureSet_d */
  /* status variable */
  NVM_Status_t status = gNVM_OK_c;

  /* search for a full table entry that owns the record */
  tblEntryMetaAddress = NvGetTblEntryMetaAddrFromId(srcMetaAddr, pSrcMetaInf->fields.NvmDataEntryID);

  if(tblEntryMetaAddress != 0)
  {
    /* found it */                
    metaAddress = srcMetaAddr;        

    /* reset the copied records index */
    recordsCopiedCurrentIdx = 0;

    /* clear the records copied buffer */
    for(loopIdx = 0; loopIdx < (uint8_t)gNvRecordsCopiedBufferSize_c; loopIdx++)
    {            
      maNvRecordsCpyIdx[loopIdx] = (uint16_t)gNvInvalidElementIndex_c;        
    }

    while(metaAddress > tblEntryMetaAddress)
    {                    
      /* get meta information */
      NvGetMetaInfo(mNvActivePageId, metaAddress, &metaInfo);

      /* skip invalid entries and full table records */
      if((metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte) || 
          (metaInfo.fields.NvValidationStartByte != gValidationByteSingleRecord_c))
      {
        metaAddress -= sizeof(NVM_RecordMetaInfo_t);
        continue;
      }                        

      if(metaInfo.fields.NvmDataEntryID == pSrcMetaInf->fields.NvmDataEntryID) /* found it */
      {   
        /* check if record is already copied */
        recordIsCopied = FALSE;

        for(loopIdx = 0; loopIdx < (uint16_t)gNvRecordsCopiedBufferSize_c; loopIdx++)
        {
          if(metaInfo.fields.NvmElementIndex == maNvRecordsCpyIdx[loopIdx])
          {
            recordIsCopied = TRUE;
            break;
          }                
        }                                
        if(recordIsCopied)
        {
          /* skip elements already copied */
          metaAddress -= sizeof(NVM_RecordMetaInfo_t);                    
          continue;
        }                

        /* check if the element still belongs to an valid RAM table entry */
        if(metaInfo.fields.NvmElementIndex >= pNVM_DataTable[srcTblEntryIdx].ElementsCount)
        {
          /* the FLASH element is no longer a current RAM table entry element */
          metaAddress -= sizeof(NVM_RecordMetaInfo_t);
          continue;
        }

        /* get element size */
        elemSize = pNVM_DataTable[srcTblEntryIdx].ElementSize;

        /* make a copy not to alter the original value and compute destination address where the element will be written */
        destination = dstRecordAddr + (metaInfo.fields.NvmElementIndex * elemSize);

        /* check alignment and adjust it if necessary */                            
        misalignedBytes = destination - (destination & (~0x03uL));                            

        /* check if the destination is longword aligned or not */
        if(misalignedBytes)
        {
          /* align to previous 32 bit boundary */
          destination &= ~0x03uL;                    
        }

        innerOffset = 0;
        
        /* compute the loop end */
        if(elemSize < (4-misalignedBytes))
        {
          loopEnd = elemSize; 
        }
        else
        {
          loopEnd = 4 - misalignedBytes;
        }

        while(elemSize)
        {
          /* read (destination) */
          NV_FlashRead(destination, (uint8_t*)&dstBuffer[0], 4);

          /* read (source) */
          NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset + innerOffset, 
                  (uint8_t*)&srcBuffer[0], loopEnd);

          /* modify */
          for(loopIdx = 0; loopIdx < loopEnd; loopIdx++, misalignedBytes++)
          {
            dstBuffer[misalignedBytes] = srcBuffer[loopIdx];
          }

          /* write (destination) */
          if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, destination, 4, dstBuffer, gFlashLaunchCommand))
          {                    
            elemSize -= loopEnd;
            innerOffset += loopEnd;
            destination += 4;
            misalignedBytes = 0;            
            if(elemSize >= 4)
            {
              loopEnd = 4;                                          
            }
            else
            {
              loopEnd = elemSize;
            }
          }
          else
          {
            return gNVM_RecordWriteError_c; 
          }
        }

        /* save the copied record offset */
        maNvRecordsCpyIdx[recordsCopiedCurrentIdx] = metaInfo.fields.NvmElementIndex;
        /* increment (and wrap if necessary) the index */
        recordsCopiedCurrentIdx = (recordsCopiedCurrentIdx + 1) & ((unsigned char) (gNvRecordsCopiedBufferSize_c - 1));                        
      }

      /* continue searching */
      metaAddress -= sizeof(NVM_RecordMetaInfo_t);            
    }

    /* 
     * now copy the elements from table entry, except the one already copied from single records 
     */

    NvGetMetaInfo(mNvActivePageId, tblEntryMetaAddress, &metaInfo);

#if gNvUseExtendedFeatureSet_d    
    fillFromRAM = FALSE;

    if(mNvTableUpdated) /* RAM table was updated */
    {                           
      if(NvGetTableEntry(pNVM_DataTable[srcTblEntryIdx].DataEntryID, &flashDataEntry))
      {
        if(pNVM_DataTable[srcTblEntryIdx].ElementsCount > flashDataEntry.ElementsCount)
        {
          /* fill the FLASH destination page with the default RAM value for the missing element(s) */                 
          fillFromRAM = TRUE;
        }
      }
    }
#endif /* gNvUseExtendedFeatureSet_d */    

    for(recordIdx = 0; recordIdx < pNVM_DataTable[srcTblEntryIdx].ElementsCount; recordIdx++)
    {
#if gNvUseExtendedFeatureSet_d      
      if(mNvTableUpdated)
      {
        if(recordIdx >= flashDataEntry.ElementsCount)
          break;
      }
#endif
      
      recordIsCopied = FALSE;

      for(loopIdx = 0; loopIdx < (uint16_t)gNvRecordsCopiedBufferSize_c; loopIdx++)
      {
        if(recordIdx == maNvRecordsCpyIdx[loopIdx])
        {
          recordIsCopied = TRUE;
          break;
        }        
      }
      if(recordIsCopied)
      {
        /* skip already copied elements */
        continue;
      }                        

      /* 
       * copy the element 
       */

       /* refresh the element size */
       elemSizeCopy = elemSize = pNVM_DataTable[srcTblEntryIdx].ElementSize;
       /* make a copy not to alter the original value and compute destination address where the element will be written */
       destination = dstRecordAddr + (recordIdx * elemSize);
       /* check alignment and adjust it if necessary */                            
       misalignedBytes = destination - (destination & (~0x03uL));                            
       /* check if the destination is longword aligned or not */
       if(misalignedBytes)
       {
         /* align to previous 32 bit boundary */
         destination &= ~0x03uL;              
       }

       innerOffset = 0;
       
       /* compute the loop end */
       if(elemSizeCopy < (4-misalignedBytes))
       {
         loopEnd = elemSizeCopy; 
       }
       else
       {
         loopEnd = 4 - misalignedBytes;
       }

       while(elemSizeCopy)
       {
         /* read (destination) */
         NV_FlashRead(destination, (uint8_t*)&dstBuffer[0], 4);

         /* read (FLASH source) */
         NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset + 
                 (recordIdx * elemSize) + innerOffset, (uint8_t*)&srcBuffer[0], loopEnd);                                     

         /* modify */
         for(loopIdx = 0; loopIdx < loopEnd; loopIdx++, misalignedBytes++)
         {
           dstBuffer[misalignedBytes] = srcBuffer[loopIdx];
         }

         /* write (destination) */
         if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, destination, 4, dstBuffer, gFlashLaunchCommand))
         {                    
           elemSizeCopy -= loopEnd;
           innerOffset += loopEnd;
           destination += 4;
           misalignedBytes = 0;
           if(elemSizeCopy >= 4)
           {
             loopEnd = 4;                                           
           }
           else
           {
             loopEnd = elemSizeCopy;
           }
         }
         else
         {
           return gNVM_RecordWriteError_c;         
         }
       }

       /* save the copied record offset */
       maNvRecordsCpyIdx[recordsCopiedCurrentIdx] = recordIdx;
       /* increment (and wrap if necessary) the index */
       recordsCopiedCurrentIdx = (recordsCopiedCurrentIdx + 1) & ((unsigned char) (gNvRecordsCopiedBufferSize_c - 1));
    }
    
#if gNvUseExtendedFeatureSet_d
    /* append the elements that were not previously stored in FLASH memory, if any */
    if(fillFromRAM)
    {
      for(; recordIdx < pNVM_DataTable[srcTblEntryIdx].ElementsCount; recordIdx++)
      {           
        /* refresh the element size */
        elemSizeCopy = elemSize = pNVM_DataTable[srcTblEntryIdx].ElementSize;
        
        /* make a copy not to alter the original value and compute destination address where the element will be written */
        destination = dstRecordAddr + (recordIdx * elemSize);

        /* check alignment and adjust it, if necessary */                            
        misalignedBytes = destination - (destination & (~0x03uL));                            

        /* check if the destination is longword aligned or not */
        if(misalignedBytes)
        {
          /* align to previous 32 bit boundary */
          destination &= ~0x03uL;             
        }

        innerOffset = 0;
        
        /* compute the loop end */
        if(elemSizeCopy < (4-misalignedBytes))
        {
          loopEnd = elemSizeCopy; 
        }
        else
        {
          loopEnd = 4 - misalignedBytes;
        }

        while(elemSizeCopy)
        {
          /* read (destination) */
          NV_FlashRead(destination, (uint8_t*)&dstBuffer[0], 4);

          /* read (RAM source) */
          NV_FlashRead((uint32_t)((uint8_t*)pNVM_DataTable[srcTblEntryIdx].pData + (recordIdx * elemSize) + innerOffset), 
                    (uint8_t*)&srcBuffer[0], loopEnd);

          /* modify */
          for(loopIdx = 0; loopIdx < loopEnd; loopIdx++, misalignedBytes++)
          {
            dstBuffer[misalignedBytes] = srcBuffer[loopIdx];
          }

          /* write (destination) */
          if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, destination, 4, dstBuffer, gFlashLaunchCommand))
          {                    
            elemSizeCopy -= loopEnd;
            innerOffset += loopEnd;
            destination += 4;
            misalignedBytes = 0;
            if(elemSizeCopy >= 4)
            {
              loopEnd = 4;                                          
            }
            else
            {
              loopEnd = elemSizeCopy;
            }
          }
          else
          {
            return gNVM_RecordWriteError_c;                      
          }
        }
        /* save the copied record offset */
        maNvRecordsCpyIdx[recordsCopiedCurrentIdx] = recordIdx;
        /* increment (and wrap if necessary) the index */
        recordsCopiedCurrentIdx = (recordsCopiedCurrentIdx + 1) & ((unsigned char) (gNvRecordsCopiedBufferSize_c - 1));
      }
    }
#endif /* gNvUseExtendedFeatureSet_d */    

    /* write meta information tag */
    dstMetaInfo.fields.NvValidationStartByte = gValidationByteAllRecords_c;
    dstMetaInfo.fields.NvmDataEntryID = pSrcMetaInf->fields.NvmDataEntryID;
    dstMetaInfo.fields.NvmElementIndex = 0;
    dstMetaInfo.fields.NvmRecordOffset = dstRecordAddr - mNvVirtualPageProperty[(mNvActivePageId+1)%2].NvRawSectorStartAddress;
    dstMetaInfo.fields.NvValidationEndByte = gValidationByteAllRecords_c;

    /* write the associated record meta information */                
    if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, dstMetaAddr, sizeof(NVM_RecordMetaInfo_t), (uint8_t*)(&dstMetaInfo), gFlashLaunchCommand))
    {
      return gNVM_MetaInfoWriteError_c;                    
    }
  }
  else
  {
    /* an entire table entry was not found, so copy the record as it is */
    if((status = NvInternalCopy(dstRecordAddr, dstMetaAddr, pSrcMetaInf, srcTblEntryIdx, pNVM_DataTable[srcTblEntryIdx].ElementSize)) != gNVM_OK_c)
    {
      return status;
    }
    #if gUnmirroredFeatureSet_d
    if(gNVM_NotMirroredInRam_c == pNVM_DataTable[srcTblEntryIdx].DataEntryType)
    {
        /* set the pointer to the flash data */
        ((uint8_t**)pNVM_DataTable[srcTblEntryIdx].pData)[pSrcMetaInf->fields.NvmElementIndex] = (uint8_t*)dstRecordAddr;   
    }
    #endif
  }
  return status;
}
#endif /* gNvFragmentation_Enabled_d */


/******************************************************************************
 * Name: NvCopyPage
 * Description: Copy the active page content to the mirror page. Only the 
 *              latest table entries / elements are copied. A merge operation
 *              is performed before copy if an entry has single elements 
 *              saved priori and newer than the table entry. If one or more
 *              elements were singular saved and the NV page doesn't has a
 *              full table entry saved, then the elements are copied as they
 *              are.  
 * Parameter(s): [IN] skipEntryId - the entry ID to be skipped when page
 *                                  copy is performed
 * Return: gNVM_InvalidPageID_c - if the source or destination page is not 
 *                                valid
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be 
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - page copy completed successfully
 *****************************************************************************/
static NVM_Status_t NvCopyPage
(    
#if gNvUseExtendedFeatureSet_d      
  NvTableEntryId_t skipEntryId
#else
  void
#endif
)
{
  /* source page related variables */
  uint32_t srcMetaAddress;
  NVM_RecordMetaInfo_t srcMetaInfo;
  uint16_t srcTableEntryIdx;

  /* destination page related variables */    
  uint32_t dstMetaAddress;
  #if gUnmirroredFeatureSet_d
    uint32_t firstMetaAddress;
  #endif  
  NVM_VirtualPageID_t dstPageId;
  uint32_t dstRecordAddress;

#if gNvUseExtendedFeatureSet_d
  uint16_t idx;  
  bool_t entryFound;
  NVM_DataEntry_t flashDataEntry;
#endif /* gNvUseExtendedFeatureSet_d */ 
  uint32_t bytesToCopy;

  /* status variable */
  NVM_Status_t status;

  dstPageId = (NVM_VirtualPageID_t)((mNvActivePageId+1)%2);

  /* Check if the destination page is blank. If not, erase it. */
  if(gNVM_PageIsNotBlank_c == NvVirtualPageBlankCheck(dstPageId))
  {
    status = NvEraseVirtualPage(dstPageId);
    if(gNVM_OK_c != status)
    {
      return status;        
    }
  }

#if gNvUseExtendedFeatureSet_d
  /* save the current RAM table */
  NvSaveRamTable(dstPageId);
#endif 

  /* start with the source page last record */
  srcMetaAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;
  /* initialise the destination page meta info start address */
#if gNvUseExtendedFeatureSet_d  
  dstMetaAddress = mNvVirtualPageProperty[dstPageId].NvRawSectorStartAddress + sizeof(mNvPageCounter) +
      (mNvTableSizeInRAM + (2 * sizeof(mNvTableMarker)));
#else
  dstMetaAddress = mNvVirtualPageProperty[dstPageId].NvRawSectorStartAddress + sizeof(mNvPageCounter);
#endif /* gNvUseExtendedFeatureSet_d*/
  #if gUnmirroredFeatureSet_d
    firstMetaAddress = dstMetaAddress;
  #endif  
  /* initialise the destination page record start address */
  dstRecordAddress = mNvVirtualPageProperty[dstPageId].NvRawSectorEndAddress - sizeof(mNvPageCounter) + 1;

  while(srcMetaAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter))
#if gNvUseExtendedFeatureSet_d        
          + (mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker)))
#endif 
       )
  {
    /* get current meta information */
    (void)NvGetMetaInfo(mNvActivePageId, srcMetaAddress, &srcMetaInfo);

#if gNvUseExtendedFeatureSet_d    
    /* NV RAM table has been updated */
    if(mNvTableUpdated)
    {
      idx = 0;
      entryFound = FALSE;

      /* check if the saved entry is still present in the new RAM table */
      while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
      {
        if(srcMetaInfo.fields.NvmDataEntryID == pNVM_DataTable[idx].DataEntryID)
        {
          #if gUnmirroredFeatureSet_d
          if(gNVM_NotMirroredInRam_c == pNVM_DataTable[idx].DataEntryType)
          {
              if (!NvIsNVMFlashAddress(((void**)pNVM_DataTable[idx].pData)[srcMetaInfo.fields.NvmElementIndex]))              
              {
                break; 
              }
          }
          #endif
          entryFound = TRUE;
          break;
        }
        idx++;
      }

      if(!entryFound)
      {
        /* move to the next meta info */
        srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
        continue;
      }
    }
#endif /* gNvUseExtendedFeatureSet_d */
    
    /* get table entry index */
    srcTableEntryIdx = NvGetTableEntryIndexFromId(srcMetaInfo.fields.NvmDataEntryID);

if((srcMetaInfo.fields.NvValidationStartByte != srcMetaInfo.fields.NvValidationEndByte) ||
        (srcTableEntryIdx == gNvInvalidDataEntry_c) ||
#if gNvUseExtendedFeatureSet_d
        (srcTableEntryIdx == skipEntryId) ||
#endif /* #if gNvUseExtendedFeatureSet_d */
        NvIsRecordCopied(dstPageId, &srcMetaInfo)
    )
    {
      /* go to the next meta information tag */
      srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
      continue;
    }

    if((srcMetaInfo.fields.NvValidationStartByte != gValidationByteSingleRecord_c) && 
        (srcMetaInfo.fields.NvValidationStartByte != gValidationByteAllRecords_c))
    {
      /* go to the next meta information tag */
      srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
      continue;
    }
    
    #if gUnmirroredFeatureSet_d
    if(gNVM_NotMirroredInRam_c == pNVM_DataTable[srcTableEntryIdx].DataEntryType)
    {
       if (!NvIsNVMFlashAddress(((void**)pNVM_DataTable[srcTableEntryIdx].pData)[srcMetaInfo.fields.NvmElementIndex]))              
       {
          /* go to the next meta information tag */
          srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);
          continue; 
       }
    }
    #endif
    /* compute the destination record start address */
    dstRecordAddress -= NvGetRecordFullSize(srcTableEntryIdx);

    bytesToCopy = pNVM_DataTable[srcTableEntryIdx].ElementsCount * pNVM_DataTable[srcTableEntryIdx].ElementSize;

#if gNvUseExtendedFeatureSet_d
    /* NV RAM table has been updated */
    if(mNvTableUpdated)
    {
      if(NvGetTableEntry(pNVM_DataTable[srcTableEntryIdx].DataEntryID, &flashDataEntry))
      {
        if(flashDataEntry.ElementSize != pNVM_DataTable[srcTableEntryIdx].ElementSize)
        {
          /* copying table entries with modified element size is not supported */
          return gNVM_NvTableWrongElementSize_c;
        }

        if(flashDataEntry.ElementsCount < pNVM_DataTable[srcTableEntryIdx].ElementsCount)
        {
          /* copy only the bytes that were previously written to FLASH virtual page */
          bytesToCopy = flashDataEntry.ElementsCount * flashDataEntry.ElementSize;              
        }
      }
    }
#endif /* gNvUseExtendedFeatureSet_d */    

#if gNvFragmentation_Enabled_d
    /* 
     * full table entry
     */        
    if(srcMetaInfo.fields.NvValidationStartByte == gValidationByteAllRecords_c)
    {               
      if((status = NvInternalCopy(dstRecordAddress, dstMetaAddress, &srcMetaInfo, srcTableEntryIdx, bytesToCopy)) != gNVM_OK_c)
      {
        return status;
      }
    }
    else
    {
      /* 
       * single element record
       */        
       if((status = NvInternalDefragmentedCopy(srcMetaAddress, &srcMetaInfo, srcTableEntryIdx, dstMetaAddress, dstRecordAddress)) != gNVM_OK_c)
       {
         return status;
       }            
    }
#else
    if((status = NvInternalCopy(dstRecordAddress, dstMetaAddress, &srcMetaInfo, srcTableEntryIdx, bytesToCopy)) != gNVM_OK_c)
    {
      return status;
    }
#endif /* gNvFragmentation_Enabled_d */

    /* update destination meta information address */
    dstMetaAddress += sizeof(NVM_RecordMetaInfo_t);

    /* move to the next meta info */
    srcMetaAddress -= sizeof(NVM_RecordMetaInfo_t);        
  };

  /* make a request to erase the old page */
  mNvErasePgCmdStatus.NvPageToErase = mNvActivePageId;
  mNvErasePgCmdStatus.NvSectorAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;
  mNvErasePgCmdStatus.NvErasePending = TRUE;

  /* update the the active page ID */
  mNvActivePageId = dstPageId;

  /* update the last meta info address */
  #if gUnmirroredFeatureSet_d
  if(dstMetaAddress == firstMetaAddress)
  {
     mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = gEmptyPageMetaAddress_c;
     #if gUnmirroredFeatureSet_d 
         mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = gEmptyPageMetaAddress_c;
     #endif
  }
  else
  #endif
  mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = dstMetaAddress - sizeof(NVM_RecordMetaInfo_t);

  #if gUnmirroredFeatureSet_d
  mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;
  #endif
  /* write the page counter */
  if(gNVM_OK_c == NvWritePageCounter(mNvActivePageId, (mNvPageCounter + 1)))
  {
    mNvPageCounter++;
  }

#if gNvUseExtendedFeatureSet_d
  if(mNvTableUpdated)
  {
    /* update the size of the NV table stored in FLASH */
    mNvTableSizeInFlash = NvGetTableSize(gFLASHTable_c);

    /* clear the flag */
    mNvTableUpdated = FALSE;
  }
#endif /* gNvUseExtendedFeatureSet_d */
  
  return gNVM_OK_c;
}


/******************************************************************************
 * Name: NvWritePageCounter
 * Description: Write the page counter value              
 * Parameter(s): [IN] pageId - the ID of the page 
 *               [IN] value - the page counter value that will written to 
 *                            the specified page
 *               
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_Error_c - if the format operation fails
 *****************************************************************************/
static NVM_Status_t NvWritePageCounter
(
  NVM_VirtualPageID_t pageId,
  uint32_t value    
)
{
  /* write page counter on page top and page bottom */
  if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, mNvVirtualPageProperty[pageId].NvRawSectorStartAddress, 
                             sizeof(value), (uint8_t*)&value, gFlashLaunchCommand))
  {            
    if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, (mNvVirtualPageProperty[pageId].NvRawSectorEndAddress - sizeof(value) + 1), 
                               sizeof(value), (uint8_t*)&value, gFlashLaunchCommand))
    {                        
      return gNVM_OK_c;
    }
    return gNVM_Error_c;
  }
  return gNVM_Error_c;
}


/******************************************************************************
 * Name: NvInternalFormat
 * Description: Format the NV storage system. The function erases in place both
 *              virtual pages and then writes the page counter value to first  
 *              virtual page. The provided page counter value is automatically 
 *              incremented and then written to first (active) virtual page.              
 * Parameter(s): [IN] pageCounterValue - the page counter value that will
 *                                       be incremented and then written to
 *                                       active page
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *****************************************************************************/
static NVM_Status_t NvInternalFormat
(
  uint32_t pageCounterValue
)
{    
  uint8_t retryCount = gNvFormatRetryCount_c;

  /* increment the page counter value */
  if(pageCounterValue == (uint32_t)gPageCounterMaxValue_c - 1)
  {
    pageCounterValue = 1;
  }
  else
  {
    pageCounterValue++;
  }

  while(retryCount--)
  {
    uint32_t status;
    /* erase first page */
    status = NV_FlashEraseSector(&gFlashConfig, 
                                 mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress,
                                 mNvVirtualPageProperty[gFirstVirtualPage_c].NvTotalPageSize,
                                 gFlashLaunchCommand);
    #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVPageEraseMonitoring(mNvVirtualPageProperty[gFirstVirtualPage_c].NvRawSectorStartAddress, status);
    #endif

    /* erase the second page */
    status = NV_FlashEraseSector(&gFlashConfig, 
                                 mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress,
                                 mNvVirtualPageProperty[gSecondVirtualPage_c].NvTotalPageSize,
                                 gFlashLaunchCommand);
    #if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
    FSCI_MsgNVPageEraseMonitoring(mNvVirtualPageProperty[gSecondVirtualPage_c].NvRawSectorStartAddress, status);
    #endif
    (void)status;

    if( (gNVM_OK_c == NvVirtualPageBlankCheck(gFirstVirtualPage_c)) && gNVM_OK_c == NvVirtualPageBlankCheck(gSecondVirtualPage_c))
    {
      break;
    }
  }

  /* write page counter on page top and page bottom */
  if(FTFx_OK == NvWritePageCounter(gFirstVirtualPage_c, pageCounterValue))
  {
    /* active page after format = first virtual page */
    mNvActivePageId = gFirstVirtualPage_c;
#if gNvUseExtendedFeatureSet_d    
    /* save NV table from RAM memory to FLASH memory */
    NvSaveRamTable(mNvActivePageId);
    /* update the size of the NV table stored in FLASH */
    mNvTableSizeInFlash = NvGetTableSize(gFLASHTable_c);
#endif /* gNvUseExtendedFeatureSet_d */
    /* update the page counter value */
    mNvPageCounter = pageCounterValue; 

    return gNVM_OK_c;
  }

  return gNVM_FormatFailure_c;
}

#if gNvUseExtendedFeatureSet_d
/******************************************************************************
 * Name: NvSaveRamTable
 * Description: Saves the NV table
 * Parameter(s): [IN] pageId - the virtual page ID where the table will be 
 *                             saved 
 * Return: TRUE if table saved successfully, FALSE otherwise
 ******************************************************************************/
static bool_t NvSaveRamTable
(
  NVM_VirtualPageID_t pageId
)
{
  uint16_t idx;
  uint32_t addr;
  uint32_t tblQual;
  uint32_t tmp;

  if(NULL == pNVM_DataTable)
    return FALSE;

  /* write table qualifier start */

  tblQual = (uint32_t)gNvTableMarker_c;
  addr = mNvVirtualPageProperty[pageId].NvRawSectorStartAddress
      + sizeof(mNvPageCounter);

  if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, addr, sizeof(uint32_t),
      (uint8_t*)(&tblQual), gFlashLaunchCommand))
  {
    return FALSE;
  }

  idx = 0;
  addr += sizeof(tblQual);

  while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
  {
    /* write data entry ID */
    tmp = (pNVM_DataTable[idx].DataEntryID << 16) + 0xFFFF;
    if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, addr, sizeof(tmp), (uint8_t*)(&tmp), gFlashLaunchCommand))
    {
      return FALSE;
    }
    /* increment address */
    addr += sizeof(tmp);

    /* write element count and element size */
    tmp = (pNVM_DataTable[idx].ElementsCount << 16) + pNVM_DataTable[idx].ElementSize;
    if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, addr, sizeof(tmp), (uint8_t*)(&tmp), gFlashLaunchCommand))
    {
      return FALSE;
    }
    /* increment address */
    addr += sizeof(tmp);

    /* increment table entry index */
    idx++;
  }

  /* write table qualifier end */
  if(FTFx_OK != NV_FlashProgramUnaligned(&gFlashConfig, addr, sizeof(uint32_t),
      (uint8_t*)(&tblQual), gFlashLaunchCommand))
  {
    return FALSE;
  }
  return TRUE;
}

/******************************************************************************
 * Name: NvGetTableSize
 * Description: Retrieves the size of the NV table
 * Parameter(s): [IN] location - specifies if the size shall be the NV FLASH 
 *                               table size (gFLASHTable_c) or the NV RAM table 
 *                               size (gRAMTable_c) 
 * Return: the NV table size
 ******************************************************************************/
static uint32_t NvGetTableSize
(
  uint8_t location
)
{
  uint32_t addr;
  uint32_t size = 0;
  uint32_t data = 0;
  uint16_t idx = 0;
  bool_t tblEndFound;

  tblEndFound = FALSE;

  if(gRAMTable_c == location)
  {
    /* compute the size of the table stored in RAM memory, except the data pointer size
     * (the data pointer is not saved into Flash memory)
     */
    while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
    {
      size += (sizeof(NVM_DataEntry_t)-sizeof(pNVM_DataTable[idx].pData));
      idx++;
    }
  }
  else
  {
    /* compute the size of the table stored in Flash memory */
    addr = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress
        + sizeof(mNvPageCounter);

    NV_FlashRead(addr, (uint8_t*)&data, sizeof(data));

    if(gNvTableMarker_c != data)
    {
      return 0;
    }

    addr += sizeof(data);

    do
    {
      NV_FlashRead(addr, (uint8_t*)&data, sizeof(data));
      if(gNvTableMarker_c == data)
      {
        tblEndFound = TRUE;
        break;
      }
      size += sizeof(data);
      addr += sizeof(data);
    } while(addr < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress);

    if(tblEndFound)
    {
      return size;
    }
    return 0;
  }
  return size;
}

/******************************************************************************
 * Name: NvIsRamTableUpdated
 * Description: Checks if the the NV table from RAM memory has changed since
 *              last system reset (e.g. via an OTA transfer)
 * Parameter(s): -
 * Return: TRUE if the NV RAM table has been changed / FALSE otherwise
 ******************************************************************************/
static bool_t NvIsRamTableUpdated
(
  void
)
{
  uint16_t idx;
  uint32_t data;
  uint32_t addr;
  uint32_t endAddr;
  bool_t idFound;

  /* address = page raw sector start address + page counter size + table marker */
  addr = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress
      + sizeof(mNvPageCounter) + sizeof(data);

  /* compute the search end address */
  endAddr = addr + mNvTableSizeInFlash;

  do
  {
    /* read ID */
    NV_FlashRead(addr, (uint8_t*)&data, sizeof(data));

    idFound = FALSE;    
    idx = 0;

    while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
    {
      if(data == ((pNVM_DataTable[idx].DataEntryID <<16) + 0xFFFF))
      {
        idFound = TRUE;
        break;
      }

      /* increment the index */
      idx++;
    }

    if(!idFound)
    {
      return TRUE; 
    }

    /* read element count and element size */
    addr += sizeof(data);
    NV_FlashRead(addr, (uint8_t*)&data, sizeof(data));

    if(data != ((pNVM_DataTable[idx].ElementsCount << 16) +
        pNVM_DataTable[idx].ElementSize)) {
      return TRUE;
    }

    /* increment the address */
    addr += sizeof(data);   

  } while(addr < endAddr);

  return FALSE;
}

/******************************************************************************
 * Name: NvGetTableEntry
 * Description: get the NV table entry information stored on FLASH memory
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pDataEntry - a pointer to a memory location where the 
 *                                  entry information will be stored
 * Return: TRUE if the has been found / FALSE otherwise
 ******************************************************************************/
static bool_t NvGetTableEntry
(
  uint16_t tblEntryId,
  NVM_DataEntry_t* pDataEntry
)
{
  uint32_t addr;
  uint32_t data;  

  pDataEntry->pData = NULL; /* the data pointer is not saved on FLASH table and 
   * shall not be used by the caller of this function */

  addr = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress
      + sizeof(mNvPageCounter);

  NV_FlashRead(addr, (uint8_t*)&data, sizeof(data));

  if(data != (uint32_t)gNvTableMarker_c)
  {   
    pDataEntry->ElementsCount = 0;
    pDataEntry->ElementSize = 0;
    pDataEntry->DataEntryID = gNvInvalidDataEntry_c;
    return FALSE;
  }

  /* increment address */
  addr += sizeof(data);

  do
  {
    NV_FlashRead(addr, (uint8_t*)&data, sizeof(data));

    if(data == (uint32_t)gNvTableMarker_c)
    {
      /* reached end of table */
      break; 
    }

    if(((data>>16) & 0x0000FFFF) == tblEntryId)
    {     
      /* read the next 4 bytes to get element count and element size */
      addr += sizeof(data);
      NV_FlashRead(addr, (uint8_t*)&data, sizeof(data));      
      pDataEntry->ElementsCount = (uint16_t)((data>>16)& 0x0000FFFF);
      pDataEntry->ElementSize = (uint16_t)(data & 0x0000FFFF);
      pDataEntry->DataEntryID = tblEntryId;
      return TRUE;
    }

    /* continue searching */
    addr += (2 * sizeof(data));

  } while(addr < mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress);

  pDataEntry->pData = NULL;
  pDataEntry->ElementsCount = 0;
  pDataEntry->ElementSize = 0;
  pDataEntry->DataEntryID = gNvInvalidDataEntry_c;

  return FALSE;
}

/******************************************************************************
 * Name: NvIsNVMFlashAddress
 * Description: check if the address is in Flash
 * Parameter(s): [IN] address
 *               
 * Return: TRUE if the table entry is in Flash / FALSE otherwise
 ******************************************************************************/
#if gUnmirroredFeatureSet_d
static bool_t NvIsNVMFlashAddress
(
  void* address
)
{
  uint8_t idx;
  for(idx=0; idx < gNvVirtualPagesCount_c; idx ++)
  {
    if( ((uint32_t)address >= mNvVirtualPageProperty[idx].NvRawSectorStartAddress) &&
        ((uint32_t)address <= mNvVirtualPageProperty[idx].NvRawSectorEndAddress))
    {
      return TRUE;
    }
  }
  return FALSE;
}
#endif

#endif /* gNvUseExtendedFeatureSet_d */

#endif /* no FlexNVM */


/******************************************************************************
 * Name: NvGetEntryFromDataPtr
 * Description: get table and element indexes based on a generic pointer address
 * Parameter(s): [IN] pData - a pointer to a NVM RAM table
 *               [OUT] pIndex - a pointer to a memory location where the 
 *                              requested indexed will be stored
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PointerOutOfRange_c - if the provided pointer cannot be founded
 *                                    within the RAM table
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvGetEntryFromDataPtr
(
  void* pData,
  NVM_TableEntryInfo_t* pIndex
)
{        
  uint16_t idx = 0;

  while(gNvEndOfTableId_c != pNVM_DataTable[idx].DataEntryID)
  {
    #if gUnmirroredFeatureSet_d
    if(gNVM_NotMirroredInRam_c == pNVM_DataTable[idx].DataEntryType)
    {
      uint8_t i;
      uint8_t** ppNVMDataTable = pNVM_DataTable[idx].pData;
      for(i=0 ; i < pNVM_DataTable[idx].ElementsCount; i++)
      {
         
        if(pData == ppNVMDataTable)
        {
           pIndex->entryId = pNVM_DataTable[idx].DataEntryID;
           pIndex->elementIndex = i;
           return gNVM_OK_c;
        }
        ppNVMDataTable++;
      } 
    } 
    else
    #endif   
    if(((uint8_t*)pData >= (uint8_t*)pNVM_DataTable[idx].pData) && ((uint8_t*)pData < ((uint8_t*)pNVM_DataTable[idx].pData + 
        (pNVM_DataTable[idx].ElementSize * pNVM_DataTable[idx].ElementsCount))))
    {
      pIndex->entryId = pNVM_DataTable[idx].DataEntryID;
      pIndex->elementIndex = (((uint32_t)pData - (uint32_t)pNVM_DataTable[idx].pData)/(pNVM_DataTable[idx].ElementSize));
      return gNVM_OK_c;
    }
    /* increment the loop counter */
    idx++;
  }    
  return gNVM_PointerOutOfRange_c;
}


/******************************************************************************
 * Name: NvWriteRecord
 * Description: writes a record
 * Parameter(s): [IN] tblIndexes - a pointer to table and element indexes
 * Return: gNVM_InvalidPageID_c - if the active page is not valid
 *         gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_MetaInfoWriteError_c - if the meta information couldn't be 
 *                                     written
 *         gNVM_RecordWriteError_c - if the record couldn't be written
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvWriteRecord
(        
  NVM_TableEntryInfo_t* tblIndexes
)
{
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */  
  uint32_t metaInfoAddress;
  uint32_t lastRecordAddress;
  uint32_t newRecordAddress;
  NVM_RecordMetaInfo_t metaInfo;        
  uint32_t realRecordSize;
  uint32_t totalRecordSize; /* record + meta */    
  uint32_t pageFreeSpace;
  uint8_t paddingBytes;
  bool_t doWrite;    
  uint32_t srcAddress;
#else /* FlexNVM */
  uint32_t lastFlexMetaInfoAddress;
  NVM_FlexMetaInfo_t lastFlexMetaInfo;
  NVM_FlexMetaInfo_t flexMetaInfo;
  uint32_t destRecordEndAddress;

#endif

  uint16_t tableEntryIdx;
  uint32_t recordSize;

  tableEntryIdx = NvGetTableEntryIndexFromId(tblIndexes->entryId);

  if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
  {
    return gNVM_InvalidTableEntry_c;
  }

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */

  recordSize = pNVM_DataTable[tableEntryIdx].ElementsCount * pNVM_DataTable[tableEntryIdx].ElementSize;

  NvGetFlexMetaInfoFromId(tblIndexes->entryId, &flexMetaInfo);

  if(flexMetaInfo.rawValue == gNvFlexGuardValue_c) /* no meta found for this table entry ID */
  {
    /* set entry ID */
    flexMetaInfo.fields.NvDataEntryID = tblIndexes->entryId;      
    /* get last meta info tag address */
    lastFlexMetaInfoAddress = NvGetFlexLastMetaInfo();

    if(lastFlexMetaInfoAddress < gFlashConfig.EERAMBase) /* FlexRAM empty */
    {              
      flexMetaInfo.fields.NvDataOffset = gFlashConfig.EEESize - recordSize;
      destRecordEndAddress = gFlashConfig.EERAMBase + gFlashConfig.EEESize;
      lastFlexMetaInfoAddress = gFlashConfig.EERAMBase;
    }
    else
    {
      /* wait for EEPROM system to be ready */
      while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
      /* read last meta tag */
      NV_FlashRead(lastFlexMetaInfoAddress, (uint8_t*)&lastFlexMetaInfo, sizeof(NVM_FlexMetaInfo_t));
      /* compute record destination end address */
      destRecordEndAddress = gFlashConfig.EERAMBase + lastFlexMetaInfo.fields.NvDataOffset;
      /* compute record offset */
      flexMetaInfo.fields.NvDataOffset = lastFlexMetaInfo.fields.NvDataOffset - recordSize;
      /* increment the last meta info address and reused it as address of the current meta info tag */
      lastFlexMetaInfoAddress += sizeof(NVM_FlexMetaInfo_t);
    }

    /* wait for EEPROM system to be ready */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

    /* write record */
    if(FTFx_OK != EEEWrite(&gFlashConfig, destRecordEndAddress - recordSize, recordSize, ((uint8_t*)(pNVM_DataTable[tableEntryIdx].pData))))
    {
      return gNVM_RecordWriteError_c;
    }   

    /* wait for EEPROM system to be ready */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

    /* write meta */
    if(FTFx_OK != EEEWrite(&gFlashConfig, lastFlexMetaInfoAddress, sizeof(NVM_FlexMetaInfo_t), (uint8_t *)(&flexMetaInfo.rawValue)))
    {
      return gNVM_RecordWriteError_c;      
    }
  }
  else /* table entry ID already in FlexRAM, update the corresponding record */
  {
    /* wait for EEPROM system to be ready */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

    if(FTFx_OK != EEEWrite(&gFlashConfig, (uint32_t)(gFlashConfig.EERAMBase + flexMetaInfo.fields.NvDataOffset), recordSize,
                           ((uint8_t*)(pNVM_DataTable[tableEntryIdx].pData))))
    {
      return gNVM_RecordWriteError_c;
    }
  }
  /* Empty macro when nvm monitoring is not enabled */
  FSCI_MsgNVWriteMonitoring(flexMetaInfo.fields.NvDataEntryID,tblIndexes->elementIndex,tblIndexes->saveRestoreAll);
  return gNVM_OK_c;

#else /* gNvUseFlexNVM_d */
   
  #if gUnmirroredFeatureSet_d
  /* For data sets not mirrored in ram a table entry is saved separate */
  if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
  {
    tblIndexes->saveRestoreAll = FALSE;
  }
  #endif
  
  if(tblIndexes->saveRestoreAll) 
  {
    realRecordSize = recordSize = pNVM_DataTable[tableEntryIdx].ElementSize * pNVM_DataTable[tableEntryIdx].ElementsCount;        
  }
  else
  {
    realRecordSize = recordSize = pNVM_DataTable[tableEntryIdx].ElementSize;
  }

  #if gUnmirroredFeatureSet_d
  /* Check if is an erase for unmirrored dataset*/
  if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
  {
    if(NULL == ((void**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex])
    {
      realRecordSize = recordSize = 0;
    }
  }
  #endif
  
  /* get active page free space */
  NvGetPageFreeSpace(&pageFreeSpace);

  /* compute the 'real record size' taking into consideration that the FTFL controller only writes in burst of 4 bytes */    
  paddingBytes = recordSize % (uint8_t)PGM_SIZE_BYTE;
  if(paddingBytes)
  {
    realRecordSize += (uint8_t)((uint8_t)PGM_SIZE_BYTE-paddingBytes);
  }

  /* compute the total size (record + meta info) */
  totalRecordSize = realRecordSize + sizeof(NVM_RecordMetaInfo_t);

  /* check if the record fits the page's free space.  
   * one extra meta info space must be kept always free, to be able to perform the meta info search */
  if(totalRecordSize + sizeof(NVM_RecordMetaInfo_t) > pageFreeSpace) 
  {        
    /* there is no space to save the record, try to copy the current active page latest records
     * to the other page
     */
#if gNvUseExtendedFeatureSet_d    
    mNvSkipTableEntryId = gNvCopyAll_c;
#endif /* #if gNvUseExtendedFeatureSet_d */
    mNvCopyOperationIsPending = TRUE;
    return gNVM_PageCopyPending_c;        
  }
  else
  {        
    metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;

    if(gEmptyPageMetaAddress_c == metaInfoAddress) 
    {      
      /* empty page, first write */

      /* set new record address */
      newRecordAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress - sizeof(mNvPageCounter) - realRecordSize + 1;

      /* set associated meta info */            
      if(tblIndexes->saveRestoreAll)
      {
        metaInfo.fields.NvValidationStartByte = gValidationByteAllRecords_c;
        metaInfo.fields.NvValidationEndByte = gValidationByteAllRecords_c;
      }
      else
      {
        metaInfo.fields.NvValidationStartByte = gValidationByteSingleRecord_c;
        metaInfo.fields.NvValidationEndByte = gValidationByteSingleRecord_c;
      }

      metaInfo.fields.NvmDataEntryID = pNVM_DataTable[tableEntryIdx].DataEntryID;
      metaInfo.fields.NvmElementIndex = tblIndexes->elementIndex;
      metaInfo.fields.NvmRecordOffset = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorEndAddress - 
          mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress - realRecordSize - sizeof(mNvPageCounter) + 1;

      /* gEmptyPageMetaAddress_c is not a valid address and it is used only as an empty page marker;
       * therefore, set the valid value of meta information address */
#if gNvUseExtendedFeatureSet_d      
      metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter) +
          mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker));
       #if gUnmirroredFeatureSet_d
        mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = metaInfoAddress;
      #endif
#else
      metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter);
#endif /* gNvUseExtendedFeatureSet_d */
    }
    else    
    {
      /* get the meta information of the last successfully written record */
      NvGetMetaInfo(mNvActivePageId, metaInfoAddress, &metaInfo);
      #if gUnmirroredFeatureSet_d
      {
        NVM_RecordMetaInfo_t metaInfoUnerased;    
        NvGetMetaInfo(mNvActivePageId, mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress, &metaInfoUnerased);
        lastRecordAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfoUnerased.fields.NvmRecordOffset;
      }
      #else
      /* get the last record start address (the address is always 4-bytes aligned) */
      lastRecordAddress = mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset;
      #endif
      /* set new record address */
      newRecordAddress = lastRecordAddress - realRecordSize;

      /* set associated meta info */
      if(tblIndexes->saveRestoreAll)
      {
        metaInfo.fields.NvValidationStartByte = gValidationByteAllRecords_c;
        metaInfo.fields.NvValidationEndByte = gValidationByteAllRecords_c;
      }
      else
      {
        metaInfo.fields.NvValidationStartByte = gValidationByteSingleRecord_c;
        metaInfo.fields.NvValidationEndByte = gValidationByteSingleRecord_c;
      }
      metaInfo.fields.NvmDataEntryID = pNVM_DataTable[tableEntryIdx].DataEntryID;
      metaInfo.fields.NvmElementIndex = tblIndexes->elementIndex;
      metaInfo.fields.NvmRecordOffset = newRecordAddress - mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;
      metaInfoAddress += sizeof(NVM_RecordMetaInfo_t);
      #if gUnmirroredFeatureSet_d
      if(realRecordSize)
      {
        mNvVirtualPageProperty[mNvActivePageId].NvLastMetaUnerasedInfoAddress = metaInfoAddress;
      }
      #endif
    }       

    /* check if the space needed by the record is really free (erased).
     * this check is necessary because it may happens that a record to be successfully written,
     * but the system fails (e.g. POR) before the associated meta information has been written.
     * the theoretically free space is computed as the difference between the last meta info 
     * address and the start address of the last successfully written record. This information
     * is valuable but may not reflect the reality, as mentioned in the explanation above */

    doWrite = FALSE;

    while(totalRecordSize + sizeof(NVM_RecordMetaInfo_t) < pageFreeSpace)
    {
      if(!NvIsMemoryAreaAvailable(newRecordAddress, realRecordSize))
      {
        /* the memory space is not blank */
        pageFreeSpace -= realRecordSize;
        newRecordAddress -= realRecordSize;
      }
      else
      {
        /* the memory space is blank */
        doWrite = TRUE;
        break;
      }            
    }  

    /* update the meta info offset, if changed */
    if(newRecordAddress - mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress != metaInfo.fields.NvmRecordOffset)
    {
      metaInfo.fields.NvmRecordOffset = newRecordAddress - mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress;
    }

    /* Write the record and associated meta information */
    if(doWrite)
    {            
      #if gUnmirroredFeatureSet_d
      if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
      {
        srcAddress = (uint32_t)(uint8_t*)((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex];
      }
      else
      #endif
      if(tblIndexes->saveRestoreAll)
      {
        srcAddress = (uint32_t)((uint8_t*)(((uint8_t*)(pNVM_DataTable[tableEntryIdx]).pData)));
      }
      else
      {
        srcAddress = (uint32_t)((uint8_t*)(((uint8_t*)(pNVM_DataTable[tableEntryIdx]).pData)) + (tblIndexes->elementIndex * recordSize));
      }
      
      #if gUnmirroredFeatureSet_d
      if(FTFx_OK == (srcAddress ? NV_FlashProgramUnaligned(&gFlashConfig, newRecordAddress, recordSize, (uint8_t*)srcAddress, gFlashLaunchCommand):FTFx_OK))
      #else
      if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, newRecordAddress, recordSize, (uint8_t*)srcAddress, gFlashLaunchCommand))
      #endif
      {                         
        #if gUnmirroredFeatureSet_d
        if(0 == srcAddress)
        {
          /* It's an erased unmirrored dataset */
          metaInfo.fields.NvmRecordOffset = 0;  
        }
        #endif
        /* record successfully written, now write the associated record meta information */
        if(FTFx_OK == NV_FlashProgramUnaligned(&gFlashConfig, metaInfoAddress, sizeof(NVM_RecordMetaInfo_t), (uint8_t*)(&metaInfo), gFlashLaunchCommand))
        {      
          /* update the last record meta information */
          mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress = metaInfoAddress;                    
          /* Empty macro when nvm monitoring is not enabled */
#if (gFsciIncluded_c && gNvmEnableFSCIMonitoring_c)
          FSCI_MsgNVWriteMonitoring(metaInfo.fields.NvmDataEntryID,tblIndexes->elementIndex,tblIndexes->saveRestoreAll); 
#endif

          #if gUnmirroredFeatureSet_d
          if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
          {
            if(metaInfo.fields.NvmRecordOffset != 0)
            {
              MSG_Free((uint8_t*)((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex]);
              ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIndexes->elementIndex] = (uint8_t*)newRecordAddress;
            }
          }  
          #endif
          return gNVM_OK_c;                    
        }
        else
        {
          return gNVM_MetaInfoWriteError_c;
        }
      }
      else
      {
        return gNVM_RecordWriteError_c;
      }
    }
    else
    {
      /* there is no space to save the record, try to copy the current active page latest records
       * to the other page
       */           
#if gNvUseExtendedFeatureSet_d      
      mNvSkipTableEntryId = gNvCopyAll_c;
#endif /* gNvUseExtendedFeatureSet_d */
      mNvCopyOperationIsPending = TRUE;
      return gNVM_PageCopyPending_c;
    }
  }
#endif /* gNvUseFlexNVM_d */
}


/******************************************************************************
 * Name: NvRestoreData
 * Description: restore an element from NVM storage to its original RAM location
 * Parameter(s): [IN] tblIdx - pointer to table and element indexes
 * Return: gNVM_NullPointer_c - if the provided pointer is NULL
 *         gNVM_PageIsEmpty_c - if page is empty
 *         gNVM_Error_c - in case of error(s)
 *         gNVM_OK_c - if the operation completed successfully
 *****************************************************************************/
static NVM_Status_t NvRestoreData
(        
  NVM_TableEntryInfo_t* tblIdx
)
{
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */
  NVM_RecordMetaInfo_t metaInfo;
  uint32_t metaInfoAddress;
  bool_t restoreSingleRecord;        
  uint32_t restoreAllMetaInfoAddress;
  uint32_t loopEndAddress;
  uint16_t elemMinIdx, elemMaxIdx, loopCnt;
  NVM_Status_t status;
#else
  NVM_FlexMetaInfo_t flexMetaInfo;
  uint32_t EERamAddress;
#endif

  uint16_t tableEntryIdx;

#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */    
  
  /* get the last meta information address */
  if((metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress) == gEmptyPageMetaAddress_c)
  {
    /* blank page, no data to restore */
    return gNVM_PageIsEmpty_c;
  }    

  if(tblIdx->entryId == gNvInvalidDataEntry_c)
  {
    /* invalid table entry */
    return gNVM_InvalidTableEntry_c;
  }
#endif

  tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx->entryId);

  if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
  {
    return gNVM_InvalidTableEntry_c;
  }

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */

  /* restore data from EERAM */
  EERamAddress = gFlashConfig.EERAMBase;

  do
  {
    /* wait for EEPROM system to be ready */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
    /* read meta info tag */
    NV_FlashRead(EERamAddress, (uint8_t*)&flexMetaInfo, sizeof(flexMetaInfo));

    if(flexMetaInfo.rawValue == gNvFlexGuardValue_c) /* end of meta info space */
    {
      break;
    }

    if(tblIdx->entryId == flexMetaInfo.fields.NvDataEntryID)
    {
      if(tblIdx->saveRestoreAll)
      {
        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
        /* read all elements */
        NV_FlashRead((gFlashConfig.EERAMBase + flexMetaInfo.fields.NvDataOffset),
            (uint8_t*)(pNVM_DataTable[tableEntryIdx].pData),
            pNVM_DataTable[tableEntryIdx].ElementSize * pNVM_DataTable[tableEntryIdx].ElementsCount);
        return gNVM_OK_c;
      }
      else
      {
        /* wait for EEPROM system to be ready */
        while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));
        /* read element */
        NV_FlashRead(gFlashConfig.EERAMBase + flexMetaInfo.fields.NvDataOffset + (tblIdx->elementIndex * pNVM_DataTable[tableEntryIdx].ElementSize),
            (uint8_t*)(((uint8_t*)pNVM_DataTable[tableEntryIdx].pData) + (tblIdx->elementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)),
            pNVM_DataTable[tableEntryIdx].ElementSize);
        return gNVM_OK_c;
      } 
    }       

    /* go to next meta tag */
    EERamAddress += sizeof(flexMetaInfo);

  } while(EERamAddress < (gFlashConfig.EERAMBase + gFlashConfig.EEESize));

  return gNVM_MetaNotFound_c;

#else /* FlexNVM */    

  restoreSingleRecord = FALSE;                                    
  restoreAllMetaInfoAddress = gNvInvalidMetaInfoAddress_c;

  /*
   * If the meta info is found, the associated record is restored, 
   * otherwise the gNVM_MetaNotFound_c will be returned
   */
  status = gNVM_MetaNotFound_c;

  
  /*** restore all ***/  
  if(tblIdx->saveRestoreAll)
  {
    elemMinIdx = pNVM_DataTable[tableEntryIdx].ElementsCount;
    elemMaxIdx = 0;         
      
    /* parse meta info backwards until the element is found */
    while(metaInfoAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter)
#if gNvUseExtendedFeatureSet_d
        + mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker))
#endif /* gNvUseExtendedFeatureSet_d */
    ))
    {
      /* get the meta information */
      NvGetMetaInfo(mNvActivePageId, metaInfoAddress, &metaInfo);

      if(metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte)
      {
        /* invalid meta info, move to the previous meta info */
        metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
        continue;
      }

      if(metaInfo.fields.NvmDataEntryID == tblIdx->entryId)
      {                
        if(metaInfo.fields.NvValidationStartByte == gValidationByteSingleRecord_c)
        {
          /* A 'restore all' command has been requested but this meta refers a single element record
           * Mark the corresponding flag  
           */
          restoreSingleRecord = TRUE;
          
          /* set the search limits */
          if(metaInfo.fields.NvmElementIndex < elemMinIdx)
          {
            elemMinIdx = metaInfo.fields.NvmElementIndex;
          }
          if(metaInfo.fields.NvmElementIndex > elemMaxIdx)
          {
            elemMaxIdx = metaInfo.fields.NvmElementIndex;
          }
          
          metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
          continue;
        }

        if(metaInfo.fields.NvValidationStartByte == gValidationByteAllRecords_c)
        {       
          /* restore all records */
          NV_FlashRead( (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset),
              (uint8_t*)pNVM_DataTable[tableEntryIdx].pData,
              (pNVM_DataTable[tableEntryIdx].ElementsCount * pNVM_DataTable[tableEntryIdx].ElementSize));
          /* In case we have unmirrored datasets - there is not save all feature for them*/
          restoreAllMetaInfoAddress = metaInfoAddress;
          status = gNVM_OK_c;
          break;
        }                    
      }
      /* move to the previous meta info */
      metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
    }

    /* restore singular element, if any */
    if(restoreSingleRecord)
    {                  
      /* set the loop end address */
      if(gNvInvalidMetaInfoAddress_c == restoreAllMetaInfoAddress)
      {
#if gNvUseExtendedFeatureSet_d
        loopEndAddress = (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter) +
            mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker)));
#else
        loopEndAddress = (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter));
#endif /* gNvUseExtendedFeatureSet_d */
      }
      else
      {
        loopEndAddress = restoreAllMetaInfoAddress + sizeof(NVM_RecordMetaInfo_t);
      }
      
      for(loopCnt = elemMinIdx; loopCnt <= elemMaxIdx; loopCnt++)
      {
          /* copy the meta info address */
          metaInfoAddress = mNvVirtualPageProperty[mNvActivePageId].NvLastMetaInfoAddress;
          
          while(metaInfoAddress >= loopEndAddress)
          {
              /* get the meta information */
              NvGetMetaInfo(mNvActivePageId, metaInfoAddress, &metaInfo);
              
              if(metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte)
              {
                  /* invalid meta info, move to the previous meta info */
                  metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
                  continue;
              }
              
              if(metaInfo.fields.NvValidationStartByte == gValidationByteAllRecords_c)
              {
                  /* not a single element record, continue searching */
                  metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
                  continue;
              }

              if((metaInfo.fields.NvmDataEntryID == tblIdx->entryId) && 
                      (loopCnt == metaInfo.fields.NvmElementIndex))
              {                                        
                  
                  #if gUnmirroredFeatureSet_d
                  if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
                  {
                    if(!metaInfo.fields.NvmRecordOffset)
                    {
                      ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[loopCnt]=NULL;
                    }
                    else
                    {
                    ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[loopCnt] =
                      (uint8_t*)mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset;
                    }
                  }
                  else
                  #endif  
                  /* restore the element */                
                  NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset, 
                          (uint8_t*)((uint8_t*)pNVM_DataTable[tableEntryIdx].pData + 
                                  (metaInfo.fields.NvmElementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)), 
                                  pNVM_DataTable[tableEntryIdx].ElementSize);
                  
                  status = gNVM_OK_c;
                  break;
              }

              /* move to the previous meta info */
              metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
          }       
      }      
    }

    if(gNVM_OK_c == status)
    {        
      maDatasetInfo[tableEntryIdx].saveNextInterval = FALSE;
      maDatasetInfo[tableEntryIdx].countsToNextSave = gNvCountsBetweenSaves_c;
    }

    return status;
  }

  /*** restore single ***/

  /* parse meta info backwards until the element is found */
  while(metaInfoAddress >= (mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + sizeof(mNvPageCounter))
#if gNvUseExtendedFeatureSet_d        
      + mNvTableSizeInFlash + (2 * sizeof(mNvTableMarker))
#endif /* gNvUseExtendedFeatureSet_d */
      )
  {
    /* get the meta information */
    NvGetMetaInfo(mNvActivePageId, metaInfoAddress, &metaInfo);

    if(metaInfo.fields.NvValidationStartByte != metaInfo.fields.NvValidationEndByte)
    {
      /* invalid meta info, move to the previous meta info */
      metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
      continue;
    }

    if(metaInfo.fields.NvmDataEntryID == tblIdx->entryId)
    {                
      if(metaInfo.fields.NvValidationStartByte == gValidationByteSingleRecord_c)
      {                
        #if gUnmirroredFeatureSet_d
        if(gNVM_NotMirroredInRam_c == pNVM_DataTable[tableEntryIdx].DataEntryType)
        {
            if(!metaInfo.fields.NvmRecordOffset)
            {
              ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIdx->elementIndex]=NULL;
            }
            else
            {
            ((uint8_t**)pNVM_DataTable[tableEntryIdx].pData)[tblIdx->elementIndex] =
              (uint8_t*)mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset;
            }
        }
        else
        #endif  
        /* restore the element */                
        NV_FlashRead(mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset, 
            (uint8_t*)((uint8_t*)pNVM_DataTable[tableEntryIdx].pData + 
                (metaInfo.fields.NvmElementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)), 
                pNVM_DataTable[tableEntryIdx].ElementSize);
        status = gNVM_OK_c;
        break;
      }

      if(metaInfo.fields.NvValidationStartByte == gValidationByteAllRecords_c)
      {   
        /* restore the single element from the entire table entry record */
        NV_FlashRead((mNvVirtualPageProperty[mNvActivePageId].NvRawSectorStartAddress + metaInfo.fields.NvmRecordOffset + 
            (metaInfo.fields.NvmElementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)), 
            ((uint8_t*)pNVM_DataTable[tableEntryIdx].pData + (metaInfo.fields.NvmElementIndex * pNVM_DataTable[tableEntryIdx].ElementSize)),
            pNVM_DataTable[tableEntryIdx].ElementSize);
        status = gNVM_OK_c;
        break;
      }                    
    }

    /* move to the previous meta info */
    metaInfoAddress -= sizeof(NVM_RecordMetaInfo_t);
  }


  if(gNVM_OK_c == status)
  {        
    maDatasetInfo[tableEntryIdx].saveNextInterval = FALSE;
    maDatasetInfo[tableEntryIdx].countsToNextSave = gNvCountsBetweenSaves_c;
  }

  return status;

#endif /* gNvUseFlexNVM_d */    
}


/******************************************************************************
 * Name: NvGetTableEntryIndex
 * Description: get the table entry index from the provided ID
 * Parameter(s): [IN] entryId - the ID of the table entry
 * Return: table entry index of gNvInvalidTableEntryIndex_c
 *****************************************************************************/
static uint16_t NvGetTableEntryIndexFromId
(
  NvTableEntryId_t entryId 
)
{
  uint16_t loopCnt = 0;

  while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
  {
    if(pNVM_DataTable[loopCnt].DataEntryID == entryId)
    {
      return loopCnt;
    }
    /* increment the loop counter */
    loopCnt++;
  }
  return gNvInvalidTableEntryIndex_c;
}


/******************************************************************************
 * Name: NvAddSaveRequestToQueue
 * Description: Add save request to save requests queue; if the request is 
 *              already stored, ignore the current request  
 * Parameter(s): [IN] ptrTblIdx - pointer to table index
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_SaveRequestRejected_c - if the request couldn't be queued         
 ******************************************************************************/
static NVM_Status_t NvAddSaveRequestToQueue
(
  NVM_TableEntryInfo_t* ptrTblIdx
)
{
  uint8_t loopIdx;  
  bool_t isQueued;

      
  if(mNvPendingSavesQueue.EntriesCount == 0)
  {
    /* add request to queue */
    if(NvPushPendingSave(&mNvPendingSavesQueue, *ptrTblIdx))
    {        
      return gNVM_OK_c;
    }
    return gNVM_SaveRequestRejected_c;
  }

  isQueued = FALSE;
  
  /* start from the queue's head */
  loopIdx = mNvPendingSavesQueue.Head;  
  
  /* check if the request is not already stored in queue */
  while(loopIdx != mNvPendingSavesQueue.Tail)
  {
    if(ptrTblIdx->entryId == mNvPendingSavesQueue.QData[loopIdx].entryId)                
    {   
      if(mNvPendingSavesQueue.QData[loopIdx].saveRestoreAll == TRUE) /* full table entry already queued */
      {
        /* request is already queued */
        isQueued = TRUE;
        break;
      }

      /* single element from table entry is queued */
      if(ptrTblIdx->saveRestoreAll == TRUE) /* a full table entry is requested to be saved */
      {
        /* update only the flag of the already queued request */
        mNvPendingSavesQueue.QData[loopIdx].saveRestoreAll = TRUE;
        /* request is already queued */
        isQueued = TRUE;
        break;
      }

      /* The request is for a single element and the queued request is also for a single element;
       * Check if the request is for the same element. If the request is for a different element, 
       * add the new request to queue.
       */
      if(ptrTblIdx->elementIndex == mNvPendingSavesQueue.QData[loopIdx].elementIndex)
      {
        /* request is already queued */
        isQueued = TRUE;
        break;
      }

      /* add request to queue */
      if(NvPushPendingSave(&mNvPendingSavesQueue, *ptrTblIdx))
      {        
        return gNVM_OK_c;
      }                                     
      return gNVM_SaveRequestRejected_c;            
    }
    /* increment and wrap the loop index */    
    loopIdx = (loopIdx + 1)  & ((uint8_t)(gNvPendigSavesQueueSize_c - 1));
  }

  if(!isQueued)
  {
    /* push the request to save operation pending queue */
    if(NvPushPendingSave(&mNvPendingSavesQueue, *ptrTblIdx))
    {        
      return gNVM_OK_c;
    }
    return gNVM_SaveRequestRejected_c;
  }

  return gNVM_OK_c;
}

/******************************************************************************
 * Name: NvIntervalTimerCallback
 * Description: Callback function of the timer used by the NvSaveOnInterval()
 * Parameter(s): [IN] timerID - timer ID
 * Return: -
 ******************************************************************************/
static void NvIntervalTimerCallback
(
void* pV
)
{
  mNvSaveOnIntervalEvent = TRUE;
}

/******************************************************************************
 * Name: GetRandomRange
 * Description: Returns a random number between 'low' and 'high'
 * Parameter(s): [IN] low, high - generated number range
 * Return: 0..255
 ******************************************************************************/
static uint8_t GetRandomRange
(
  uint8_t low, 
  uint8_t high
)
{
  uint32_t random;

  RNG_GetRandomNo(&random);

  if(high <= low)
  {
    return low;
  }
  return low + (uint8_t)(random % (high - low + 1));    
};

#if (gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE != 0) /* FlexNVM */
/******************************************************************************
 * Name: NvGetFlexLastMetaInfo
 * Description: Get FlexRAM last meta information address 
 * Parameter(s): -
 * Return: the address of the last valid meta information       
 ******************************************************************************/
static uint32_t NvGetFlexLastMetaInfo
(
  void
)
{
  uint32_t address, size;
  NVM_FlexMetaInfo_t flexMetaInfo;

  address = gFlashConfig.EERAMBase;
  size = gFlashConfig.EEESize;

  while(size)
  {
    /* wait for EEPROM system to be ready */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY))); 
    /* read meta info tag */
    NV_FlashRead(address, (uint8_t*)&flexMetaInfo, sizeof(flexMetaInfo));
    if(flexMetaInfo.rawValue == gNvFlexGuardValue_c)
    {    
      break;
    }
    address += sizeof(flexMetaInfo);
    size -= sizeof(flexMetaInfo);
  }  
  return address - sizeof(flexMetaInfo);  
}

/******************************************************************************
 * Name: NvGetFlexMetaInfoFromId
 * Description: Get FlexRAM meta information tag from table entry ID 
 * Parameter(s): [IN] tblEntryId - table entry ID
 *               [OUT] pMetaInfo - a pointer to a memory location where the
 *                                 meta information tag will be stored
 * Return: -
 ******************************************************************************/
static void NvGetFlexMetaInfoFromId
(
  NvTableEntryId_t tblEntryId,
  NVM_FlexMetaInfo_t* pMetaInfo
)
{
  uint32_t address, size;
  NVM_FlexMetaInfo_t flexMetaInfo;

  address = gFlashConfig.EERAMBase;
  size = gFlashConfig.EEESize;

  while(size)
  {
    /* wait for EEPROM system to be ready */
    while(!(REG_BIT_GET(gFlashConfig.ftfxRegBase + FTFx_SSD_FCNFG_OFFSET, FTFx_SSD_FCNFG_EEERDY)));

    NV_FlashRead(address, (uint8_t*)&flexMetaInfo, sizeof(flexMetaInfo));
    if(flexMetaInfo.rawValue == gNvFlexGuardValue_c)
    {
      break;
    }

    if(flexMetaInfo.fields.NvDataEntryID == tblEntryId)
    {
      pMetaInfo->fields.NvDataEntryID = flexMetaInfo.fields.NvDataEntryID;
      pMetaInfo->fields.NvDataOffset = flexMetaInfo.fields.NvDataOffset;
      return;
    }

    address += sizeof(flexMetaInfo);
    size -= sizeof(flexMetaInfo);
  }

  pMetaInfo->rawValue = gNvFlexGuardValue_c;
}

/******************************************************************************
 * Name: NvCheckNvmTableForFlexRAMUsage
 * Description: Check if the existing NVM table fits within the FlexRAM window 
 * Parameter(s): -
 * Return: gNVM_NvTableExceedFlexRAMSize_c - the table exceed the size of
 *                                           FlexRAM window
 *         gNVM_OK_c - the table fits within the size of window FlexRAM window                                  
 ******************************************************************************/
static NVM_Status_t NvCheckNvmTableForFlexRAMUsage
(
  void
)
{
  index_t loopCnt = 0;
  uint32_t allDatasetSize = 0;    

  while(gNvEndOfTableId_c != pNVM_DataTable[loopCnt].DataEntryID)
  {
    if(gNvInvalidDataEntry_c == pNVM_DataTable[loopCnt].DataEntryID)
    {
      loopCnt++;
      continue;
    }  
    /* add the record size */
    allDatasetSize += (pNVM_DataTable[loopCnt].ElementsCount * pNVM_DataTable[loopCnt].ElementSize);
    /* add the meta size */
    allDatasetSize += sizeof(NVM_FlexMetaInfo_t);
    /* increment the loop counter */
    loopCnt++;
  }

  /* add the safe guard space (equal to meta size) */
  allDatasetSize += sizeof(NVM_FlexMetaInfo_t);

  if(allDatasetSize > gFlashConfig.EEESize)
  {
    return gNVM_NvTableExceedFlexRAMSize_c;  
  }  

  return gNVM_OK_c;
}

#endif /* gNvUseFlexNVM_d */

#endif /* gNvStorageIncluded_d */



/*****************************************************************************
 *****************************************************************************
 * Public functions
 *****************************************************************************
 *****************************************************************************/


/******************************************************************************
 * Name: NvModuleInit
 * Description: Initialize the NV storage module
 * Parameter(s): -
 * Return: gNVM_ModuleAlreadyInitialized_c - if the module is already 
 *                                           initialized
 *         gNVM_InvalidSectorsCount_c - if the sector count configured in the
 *                                      project linker file is invalid
 *         gNVM_MetaNotFound_c - if no meta information was found                                       
 *         gNVM_OK_c - module was successfully initialized
 *         gNVM_CannotCreateMutex_c - no mutex available
 *****************************************************************************/
NVM_Status_t NvModuleInit
(
  void
)
{
#if gNvStorageIncluded_d
  NVM_Status_t status;
  status = __NvModuleInit();
  if(status != gNVM_OK_c)
  {
    return status;
  }

  if( kStatus_OSA_Success != OSA_MutexCreate(&mNVMMutex) )
  {
    mNvModuleInitialized = FALSE; 
    return gNVM_CannotCreateMutex_c;
  }
    
#if (gFsciIncluded_c && gNvmEnableFSCIRequests_c)
  FSCI_RegisterOpGroup(gNV_FsciReqOG_d, 
                       gFsciMonitorMode_c, 
                       NV_FsciMsgHandler, 
                       NULL, 
                       gNvmDefaultFsciInterface_c);
#endif
  return gNVM_OK_c;
    
#else
  return gNVM_Error_c;
#endif /* #if gNvStorageIncluded_d */
}

/******************************************************************************
 * Name: NvMoveToRam
 * Description: Move from NVM to Ram an unmirrored dataset
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_NoMemory_c - in case there is not a memory block free
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 *****************************************************************************/

NVM_Status_t NvMoveToRam
(
  void** ppData
)
{
#if gNvStorageIncluded_d && gUnmirroredFeatureSet_d  
NVM_Status_t status; 
(void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
status = __NvmMoveToRam(ppData);
(void)OSA_MutexUnlock(&mNVMMutex);
return status;
#else
(void)ppData;
return gNVM_Error_c;
#endif 
}

/******************************************************************************
 * Name: NvErase
 * Description: Erase from NVM an unmirrored dataset 
 * Parameter(s):  ppData     double pointer to the entity to be moved from flash to RAM
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_NoMemory_c - in case there is not a memory block free
 *         Note: see also return codes of NvGetEntryFromDataPtr() function
 *****************************************************************************/

NVM_Status_t NvErase
(
  void** ppData
)
{
#if gNvStorageIncluded_d && gUnmirroredFeatureSet_d  
NVM_Status_t status; 
(void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
status = __NvmErase(ppData);
(void)OSA_MutexUnlock(&mNVMMutex);
return status;
#else
(void)ppData;
return gNVM_Error_c;
#endif 
}


/******************************************************************************
 * Name: NvSaveOnIdle
 * Description: Save the data pointed by ptrData on the next call to NvIdle()
 * Parameter(s): [IN] ptrData - pointer to data to be saved
 *               [IN] saveRestoreAll - specify if all the elements from the NVM table
 *                              entry shall be saved
 * Return: gNVM_OK_c - if operation completed successfully
 *         gNVM_Error_c - in case of error(s)
 *         Note: see also return codes of NvGetEntryFromDataPtr() function         
 ******************************************************************************/
NVM_Status_t NvSaveOnIdle
(
  void* ptrData,
  bool_t saveAll
)
{    
#if gNvStorageIncluded_d    
NVM_Status_t status;  
(void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
status = __NvSaveOnIdle(ptrData,saveAll);
(void)OSA_MutexUnlock(&mNVMMutex);
return status;
#else
  (void)ptrData;
  (void)saveAll;
  return gNVM_Error_c;
#endif /* # gNvStorageIncluded_d */
}

/******************************************************************************
 * Name: NvSaveOnInterval
 * Description:  save no more often than a given time interval. If it has 
 *               been at least that long since the last save,
 *               this function will cause a save the next time the idle 
 *               task runs.
 * Parameters: [IN] ptrData - pointer to data to be saved
 * NOTE: this function saves all the element of the table entry pointed by
 *       ptrData 
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function 
 ******************************************************************************/
NVM_Status_t NvSaveOnInterval
(
void* ptrData
)
{
#if gNvStorageIncluded_d
  NVM_Status_t status;
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  status = __NvSaveOnInterval(ptrData);
  (void)OSA_MutexUnlock(&mNVMMutex);
  return status;
#else    
  (void)ptrData;
  return gNVM_Error_c;
#endif
}                                       /* NvSaveOnInterval() */


/******************************************************************************
 * Name: NvSaveOnCount
 * Description: Decrement the counter. Once it reaches 0, the next call to 
 *              NvIdle() will save the entire table entry (all elements).
 * Parameters: [IN] ptrData - pointer to data to be saved
 * Return: NVM_OK_c - if operation completed successfully
 *         Note: see also return codes of NvGetEntryFromDataPtr() function 
 ******************************************************************************/
NVM_Status_t NvSaveOnCount
(
void* ptrData
)
{
#if gNvStorageIncluded_d
  
  NVM_Status_t status;
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  status = __NvSaveOnCount(ptrData);
  (void)OSA_MutexUnlock(&mNVMMutex);
  return status;
  
#else    
  (void)ptrData;
  return gNVM_Error_c;
#endif
}                                       /* NvSaveOnCount() */


/******************************************************************************
 * Name: NvSetMinimumTicksBetweenSaves
 * Description: Set the timer used by NvSaveOnInterval(). Takes effect after 
 *              the next save.
 * Parameters: [IN] newInterval - new time interval
 * Return: - 
 ******************************************************************************/
void NvSetMinimumTicksBetweenSaves
(
  NvSaveInterval_t newInterval
)
{
#if gNvStorageIncluded_d
  gNvMinimumTicksBetweenSaves = newInterval;
#else
  (void)newInterval;
#endif
}                                       /* NvSetMinimumTicksBetweenSaves() */


/******************************************************************************
 * Name: NvSetCountsBetweenSaves
 * Description: Set the counter trigger value used by NvSaveOnCount().
 *              Takes effect after the next save.
 * Parameters: [IN] newCounter - new counter value
 * Return: - 
 ******************************************************************************/
void NvSetCountsBetweenSaves
(
  NvSaveCounter_t newCounter
)
{
#if gNvStorageIncluded_d    
  gNvCountsBetweenSaves = newCounter;    
#else
  (void)newCounter;
#endif
}                                       /* NvSetCountsBetweenSaves() */


/******************************************************************************
 * Name: NvTimerTick
 * Description: Called from the idle task to process save-on-interval requests
 * Parameters: [IN] countTick - enable/disable tick count
 * Return: FALSE if the timer tick counters for all data sets have reached 
 *         zero. In this case, the timer can be turned off.
 *         TRUE if any of the data sets' timer tick counters have not yet
 *         counted down to zero. In this case, the timer should be active
 ******************************************************************************/
bool_t NvTimerTick
(
bool_t countTick
)
{
#if gNvStorageIncluded_d
  bool_t fTicksLeft;                
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  fTicksLeft = __NvTimerTick(countTick);
  (void)OSA_MutexUnlock(&mNVMMutex);
  return fTicksLeft;
#else
  (void)countTick;
  return FALSE;
#endif /* #if gNvStorageIncluded_d */
}                                       /* NvTimerTick() */


/******************************************************************************
 * Name: NvRestoreDataSet
 * Description: copy the most recent version of the element/table entry pointed 
 *              by ptrData from NVM storage system to RAM memory
 * Parameter(s): [IN] ptrData - pointer to data (element) to be restored 
 *               [IN] restoreAll - if FALSE restores a single element
 *                               - if TRUE restores an entire table entry
 * Return: status of the restore operation
 *****************************************************************************/
NVM_Status_t NvRestoreDataSet
(
void* ptrData,    
bool_t restoreAll
)
{
#if gNvStorageIncluded_d
  NVM_Status_t status;
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  status = __NvRestoreDataSet(ptrData,restoreAll);
  (void)OSA_MutexUnlock(&mNVMMutex);
  return status;
#else
  (void)ptrData;
  (void)restoreAll;
  return gNVM_Error_c;
#endif
}

/******************************************************************************
 * Name: NvClearCriticalSection
 * Description: leave critical section
 * Parameters: -
 * Return: - 
 ******************************************************************************/
void NvClearCriticalSection
(
  void
) 
{
#if (gNvStorageIncluded_d && gNvEnableCriticalSection_c)
  if(mNvCriticalSectionFlag)  /* in case of set/clear mismatch */
    --mNvCriticalSectionFlag;
#endif
}


/******************************************************************************
 * Name: NvSetCriticalSection
 * Description: enter critical section
 * Parameters: -
 * Return: - 
 ******************************************************************************/
void NvSetCriticalSection
(
  void
) 
{
#if (gNvStorageIncluded_d && gNvEnableCriticalSection_c)
  ++mNvCriticalSectionFlag;
#endif    
}


/******************************************************************************
 * Name: NvIdle
 * Description: Called from the idle task (bare-metal) or NVM_Task (MQX,
 *              FreeRTOS) to process the pending saves, erase or copy 
 *              operations.
 * Parameters: -
 * Return: -
 ******************************************************************************/
void NvIdle
(
void
)
{
#if gNvStorageIncluded_d
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  __NvIdle();
  (void)OSA_MutexUnlock(&mNVMMutex);
  
#endif
}                                       /* NvIdle() */


/******************************************************************************
 * Name: NvIsDataSetDirty
 * Description: return TRUE if the element pointed by ptrData is dirty
 * Parameters: [IN] ptrData - pointer to data to be checked
 * Return: TRUE if the element is dirty, FALSE otherwise 
 ******************************************************************************/
bool_t NvIsDataSetDirty
(
  void* ptrData
)
{    
#if gNvStorageIncluded_d

  NVM_TableEntryInfo_t tblIdx;
  uint16_t tableEntryIdx;

  if(!mNvModuleInitialized)
  {
    return FALSE;
  }

  if(NULL == ptrData)
  {
    return gNVM_NullPointer_c;
  }

  if(gNVM_OK_c != NvGetEntryFromDataPtr(ptrData, &tblIdx))
  {
    return FALSE;
  }
  else
  {
    tableEntryIdx = NvGetTableEntryIndexFromId(tblIdx.entryId);

    if(gNvInvalidTableEntryIndex_c == tableEntryIdx)
    {
      return FALSE;
    }

    return(maDatasetInfo[tableEntryIdx].saveNextInterval || 
        (maDatasetInfo[tableEntryIdx].countsToNextSave != gNvCountsBetweenSaves_c));         
  }

#else
  (void)ptrData;
  return FALSE;
#endif
}

/******************************************************************************
 * Name: NvGetStatistics
 * Description:       
 * Parameter(s): [OUT] ptrStat - pointer to a memory location where the pages
 *                               statistics (erase cycles of each page) will 
 *                               be stored
 * Return: -
 *****************************************************************************/
void NvGetPagesStatistics
(
  NVM_Statistics_t* ptrStat 
)
{    
#if gNvStorageIncluded_d
#if (gNvUseFlexNVM_d == FALSE) || ((gNvUseFlexNVM_d == TRUE) && (DEBLOCK_SIZE == 0)) /* no FlexNVM */    
  if(!mNvModuleInitialized)
  {
    return;
  }

  if(NULL == ptrStat)
  {
    return;
  }

  if(mNvPageCounter%2)
  {
    ptrStat->FirstPageEraseCyclesCount = ptrStat->SecondPageEraseCyclesCount = (mNvPageCounter-1)/2;
  }
  else
  {
    ptrStat->FirstPageEraseCyclesCount = mNvPageCounter/2;
    ptrStat->SecondPageEraseCyclesCount = (mNvPageCounter-2)/2;
  }

#else /* FlexNVM */
ptrStat->FirstPageEraseCyclesCount = 0;
ptrStat->SecondPageEraseCyclesCount = 0;
return;
#endif

#else
(void)ptrStat;
return;
#endif
}

/******************************************************************************
 * Name: NvFormat
 * Description: Format the NV storage system. The function erases both virtual
 *              pages and then writes the page counter to active page.              
 * Parameter(s): -
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_FormatFailure_c - if the format operation fails
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_CriticalSectionActive_c - if the system has entered in a 
 *                                        critical section
 *****************************************************************************/
NVM_Status_t NvFormat
(
void
)
{
#if gNvStorageIncluded_d
  NVM_Status_t status;
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  status = __NvFormat();   
  (void)OSA_MutexUnlock(&mNVMMutex);
  return status;
#else    
  return gNVM_Error_c;
#endif /* gNvStorageIncluded_d */    
}


/******************************************************************************
 * Name: NvRegisterTableEntry
 * Description: The function tries to register a new table entry within an
 *              existing NV table. If the NV table contained an erased (invalid)
 *              entry, the entry will be overwritten with a new one (provided
 *              by the mean of this function arguments)      
 * Parameter(s): [IN] ptrData - generic pointer to RAM data to be registered
 *                              within the NV storage system
 *               [IN] uniqueId - an unique ID of the table entry
 *               [IN] elemCount - how many elements the table entry contains
 *               [IN] elemSize - the size of an element 
 *               [IN] overwrite - if an existing table entry shall be 
 *                                overwritten
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *****************************************************************************/
NVM_Status_t NvRegisterTableEntry
(
void* ptrData,
NvTableEntryId_t uniqueId,
uint16_t elemCount,
uint16_t elemSize,
bool_t overwrite
)
{
#if gNvStorageIncluded_d && gNvUseExtendedFeatureSet_d
  
  NVM_Status_t status;
  OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);    
  status = __NvRegisterTableEntry(ptrData,uniqueId,elemCount,elemSize,overwrite);
  OSA_MutexUnlock(&mNVMMutex);
  return status;
#else
  (void)ptrData;
  (void)uniqueId;
  (void)elemCount;
  (void)elemSize;
  (void)overwrite;
  return gNVM_Error_c;
#endif
}


/******************************************************************************
 * Name: NvEraseEntryFromStorage
 * Description: The function removes a table entry within the existing NV 
 *              table. 
 * Parameter(s): [IN] ptrData - a pointer to an existing RAM data that is
 *                              managed by the NV storage system    
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *****************************************************************************/
NVM_Status_t NvEraseEntryFromStorage
(
void* ptrData
)
{
#if gNvStorageIncluded_d && gNvUseExtendedFeatureSet_d
  NVM_Status_t status;
  OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);      
  status = __NvEraseEntryFromStorage(ptrData);
  OSA_MutexUnlock(&mNVMMutex);  
  return status;
#else
  (void)ptrData;
  return gNVM_Error_c;
#endif    
}

/******************************************************************************
 * Name: NvSyncSave
 * Description: The function saves the pointed element or the entire table
 *              entry to the storage system. The save operation is not
 *              performed on the idle task but within this function call.
 * Parameter(s): [IN] ptrData - a pointer to data to be saved
 *               [IN] saveAll - specifies if the entire table entry shall be 
 *                              saved or only the pointed element
 *               [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully         
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section                             
 *****************************************************************************/
NVM_Status_t NvSyncSave
(
void* ptrData,
bool_t saveAll,
bool_t ignoreCriticalSectionFlag
)
{
#if gNvStorageIncluded_d
  NVM_Status_t status;
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  status = __NvSyncSave(ptrData,saveAll,ignoreCriticalSectionFlag);
  (void)OSA_MutexUnlock(&mNVMMutex);
  return status;
  
#else
  (void)ptrData;
  (void)saveAll;
  return gNVM_Error_c;
#endif
}


/******************************************************************************
 * Name: NvAtomicSave
 * Description: The function performs an atomic save of the entire NV table
 *              to the storage system. The operation is performed
 *              in place (atomic).
 * Parameter(s):  [IN] ignoreCriticalSectionFlag - if set to TRUE, the critical
 *                                                section flag is ignored
 * Return: gNVM_OK_c - if the operation completes successfully
 *         gNVM_ModuleNotInitialized_c - if the NVM  module is not initialized
 *         gNVM_NullPointer_c - if a NULL pointer is provided
 *         gNVM_PointerOutOfRange_c - if the pointer is out of range
 *         gNVM_InvalidTableEntry_c - if the table entry is not valid
 *         gNVM_MetaInfoWriteError_c - meta tag couldn't be written
 *         gNVM_RecordWriteError_c - record couldn't be written
 *         gNVM_CriticalSectionActive_c - the module is in critical section
 *****************************************************************************/
NVM_Status_t NvAtomicSave
(
bool_t ignoreCriticalSectionFlag
)
{
#if gNvStorageIncluded_d
  NVM_Status_t status ;
  (void)OSA_MutexLock(&mNVMMutex, OSA_WAIT_FOREVER);
  status = __NvAtomicSave(ignoreCriticalSectionFlag);
  (void)OSA_MutexUnlock(&mNVMMutex);
  return status;
#else
  return gNVM_Error_c;
#endif
}


