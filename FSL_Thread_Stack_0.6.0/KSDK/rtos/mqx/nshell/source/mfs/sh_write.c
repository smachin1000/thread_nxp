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
#include <sh_prv.h>

#include "nio.h"
#include "fcntl.h"
#include "errno.h"

#if SHELLCFG_USES_MFS
#include <mfs.h>

#include "fs_supp.h"

#define MAX_BUFF_SIZE 8192

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    :   Shell_write
* Returned Value   :  int32_t error code
* Comments  :  Reads from a file .
*
*END*---------------------------------------------------------------------*/

int32_t  Shell_write(int32_t argc, char *argv[] )
{ /* Body */
   bool           print_usage, shorthelp = FALSE;
   int32_t            return_code = SHELL_EXIT_SUCCESS;
   uint32_t           count=0;
   int32_t            offset=0;
   int32_t            seek_mode=0;
   int                open_mode=O_WRONLY | O_CREAT | O_TRUNC;
   char              c;
   int fd = -1;
   char          *abs_path;
   bool           cache_enabled=TRUE;
   SHELL_CONTEXT_PTR    shell_ptr = Shell_get_context( argv );
   char             *buf = NULL;
   int32_t               writesize = MAX_BUFF_SIZE;
   int32_t               error = 0;
   _mqx_int             result;
   _mqx_int             bi;

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      if ((argc < 2) || (argc > 5)) {
         fprintf(shell_ptr->STDOUT, "Error, invalid number of parameters\n");
         return_code = SHELL_EXIT_ERROR;
         print_usage=TRUE;
      }
      /* check if filesystem is mounted */
      else if (0 > Shell_get_current_filesystem(argv))
      {
         fprintf(shell_ptr->STDOUT, "Error, file system not mounted\n");
         return_code = SHELL_EXIT_ERROR;
      }
      else  {
         count = 0;
         offset = 0;
         seek_mode = SEEK_CUR;
         if (argc >= 3)  {
            if (! Shell_parse_uint_32(argv[2], &count ))  {
               fprintf(shell_ptr->STDOUT, "Error, invalid length\n");
               return_code = SHELL_EXIT_ERROR;
               print_usage=TRUE;
            } else  {
               if (argc >= 5)  {
                  if (strcmp(argv[3], "begin") == 0) {
                     seek_mode = SEEK_SET;
                  } else if (strcmp(argv[3], "end") == 0) {
                     seek_mode = SEEK_END;
                  } else if (strcmp(argv[3], "current") == 0) {
                     seek_mode = SEEK_CUR;
                  } else {
                     fprintf(shell_ptr->STDOUT, "Error, invalid seek type\n");
                     return_code = SHELL_EXIT_ERROR;
                     print_usage=TRUE;
                  }

                  if (return_code == SHELL_EXIT_SUCCESS)  {
                     if (! Shell_parse_int_32(argv[4], &offset ))  {
                        fprintf(shell_ptr->STDOUT, "Error, invalid seek value\n");
                        return_code = SHELL_EXIT_ERROR;
                        print_usage=TRUE;
                     }
                  }
                  open_mode = O_WRONLY | O_CREAT; /* seek is specified, do not truncate the file */
               }
            }
         }
      }

      if (return_code == SHELL_EXIT_SUCCESS)  {
         if (MFS_alloc_path(&abs_path) != MFS_NO_ERROR) {
            fprintf(shell_ptr->STDOUT, "Error, unable to allocate memory for paths\n" );
            return_code = SHELL_EXIT_ERROR;
         } else {
            error = _io_rel2abs(abs_path,shell_ptr->CURRENT_DIR,(char *) argv[1],PATHNAME_SIZE,shell_ptr->CURRENT_DEVICE_NAME);

            do {
               buf = _mem_alloc(writesize);
               if (buf != NULL) break;
               else writesize >>= 1;
            } while (writesize > 0);

            if (buf != NULL)
            {
               if(!error)
               {
                  fd = open(abs_path, open_mode);
               }
               if (0 <= fd) {
                  if (0 <= lseek(fd, offset, seek_mode)) {

                     c = '0';
                     while (count) {

                         /* generate data to buf */
                         for (bi = 0; (bi < count) && (bi < writesize); bi++) {
                             buf[bi] = c;
                             c = (c == 'z') ? '0' : c+1;
                         }

                         result = write(fd, buf, bi);

                         if (result != bi) {
                             /* incomplete write */
                             fprintf(shell_ptr->STDOUT, "Error writing file %s.\n", argv[1] );
                             error = errno;
                             if (error == MFS_DISK_FULL)
                                 fprintf(shell_ptr->STDOUT, "Disk full.\n" );
                             return_code = SHELL_EXIT_ERROR;
                             break;
                         }

                         count -= result;
                     }

                  } else {
                     fprintf(shell_ptr->STDOUT, "Error, unable to seek file %s.\n", argv[1] );
                     return_code = SHELL_EXIT_ERROR;
                  }

                  close(fd);

               } else  {
                  fprintf(shell_ptr->STDOUT, "Error, unable to open file %s.\n", argv[1] );
                  return_code = SHELL_EXIT_ERROR;
               }
               MFS_free_path(abs_path);
               _mem_free(buf);
            }
            else {
               fprintf(shell_ptr->STDOUT, "Error, unable to allocate memory for write buffer\n");
               return_code = SHELL_EXIT_ERROR;
            }
         }
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s <filename> <bytes> [<seek_mode>] [<offset>]\n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s <filename> <bytes> [<seek_mode>] [<offset>]\n", argv[0]);
         fprintf(shell_ptr->STDOUT, "   <filename>   = filename to write\n");
         fprintf(shell_ptr->STDOUT, "   <bytes>      = number of bytes to write\n");
         fprintf(shell_ptr->STDOUT, "   <seek_mode>  = one of: begin, end or current\n");
         fprintf(shell_ptr->STDOUT, "   <offset>     = seek offset\n");
      }
   }
   return return_code;
} /* Endbody */

#endif //SHELLCFG_USES_MFS
/* EOF */
