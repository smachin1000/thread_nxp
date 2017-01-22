/*HEADER**********************************************************************
*
* Copyright 2008 Freescale Semiconductor, Inc.
* Copyright 2004-2008 Embedded Access Inc.
* Copyright 1989-2008 ARC International
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains the source for an MFS shell function.
*
*
*END************************************************************************/

#include <stdio.h>
#include <string.h>
#include <mqx.h>
#include <shell.h>
#include "sh_prv.h"

#include <nio.h>
#include <fcntl.h>
#include <fs_supp.h>

#include <stdio.h>


#if SHELLCFG_USES_MFS

#include <mfs.h>
#include <sh_mfs.h>

#define SECTOR_SIZE 512


/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    :   Shell_disect
* Returned Value   :  int32_t error code
* Comments  :  Reads from a file .
*
*END*---------------------------------------------------------------------*/


int32_t  Shell_disect(int32_t argc, char *argv[] )
{ /* Body */
   int32_t            return_code = SHELL_EXIT_SUCCESS;
   bool               print_usage, shorthelp = FALSE;
   uint32_t           sector = 0;
   int                fd = -1;
   uint32_t           e,i;
   unsigned char      *buffer;
   SHELL_CONTEXT_PTR shell_ptr = Shell_get_context(argv);

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      if ((argc < 2) || (argc > 4)) {
         fprintf(shell_ptr->STDOUT, "Error, invalid number of parameters\n");
         return_code = SHELL_EXIT_ERROR;
         print_usage = TRUE;
      } else if ( !Shell_parse_uint_32(argv[1], (uint32_t *) &sector ))  {
         fprintf(shell_ptr->STDOUT, "Error, invalid sector number.\n");
         return_code = SHELL_EXIT_ERROR;
         print_usage = TRUE;
      } else {
         int fs;

         if (argc >= 3) {
            /* open the device if it was specified */
            fs = _io_get_fs_by_name(argv[2]);
         }
         else {
            /* get the current opened device descriptor */
            fs = Shell_get_current_filesystem(argv);
         }
         if (0 > fs) {
            fprintf(shell_ptr->STDOUT, "Error, unable to open disk.\n");
            return_code = SHELL_EXIT_ERROR;
         }
         if (0 > ioctl(fs, IO_IOCTL_GET_DEVICE_HANDLE, &fd)) {
            fprintf(shell_ptr->STDOUT, "Error, unable to get device descriptor.\n");
            return_code = SHELL_EXIT_ERROR;
         }

         if (0 <= fd) {
            buffer = _mem_alloc(SECTOR_SIZE);
            if (buffer) {
               if (0 > lseek(fd, sector * SECTOR_SIZE, SEEK_SET))  {
                  fprintf(shell_ptr->STDOUT, "Error, unable to seek to sector %s.\n", argv[1] );
                  return_code = SHELL_EXIT_ERROR;
               } else if (read(fd, (char *) buffer, SECTOR_SIZE) != SECTOR_SIZE) {
                  fprintf(shell_ptr->STDOUT, "Error, unable to read sector %s.\n", argv[1] );
                  return_code = SHELL_EXIT_ERROR;
               } else {
                  fprintf(shell_ptr->STDOUT, "\nSector # %d\n", sector);
                  for (e = 0; e < (32 + SECTOR_SIZE - 1) / 32; e++)  {
                     for (i = 0; i < 32; i++) {
                        fprintf(shell_ptr->STDOUT, "%02x ",(uint32_t) buffer[e*32+i]);
                     }
                     fprintf(shell_ptr->STDOUT, "\n");
                  }
               }
               _mem_free(buffer);
            }
         }

         fprintf(shell_ptr->STDOUT, "\n");
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s <sector> [<device>]\n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s <sector> [<device>]\n", argv[0]);
         fprintf(shell_ptr->STDOUT, "   <sector>     = sector number\n");
         fprintf(shell_ptr->STDOUT, "   <device>     = low level device\n");
      }
   }

   return return_code;
} /* Endbody */

#endif // SHELLCFG_USES_MFS
