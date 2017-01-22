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
#include "fs_supp.h"

#if SHELLCFG_USES_MFS
#include <mfs.h>

#define COMPARE_BLOCK_SIZE 512

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    :  Shell_compare
* Returned Value   :  uint32_t error code
* Comments  :  Renames or moves a file.
*
*END*---------------------------------------------------------------------*/

int32_t  Shell_compare(int32_t argc, char *argv[] )
{ /* Body */
   bool      print_usage, shorthelp = FALSE;
   int32_t      size1, size2, return_code = SHELL_EXIT_SUCCESS;
   int in_fd_1 = -1, in_fd_2 = -1;
   char         *file1=NULL, *file2=NULL, *file_name=NULL;
   SHELL_CONTEXT_PTR    shell_ptr = Shell_get_context( argv );
   int32_t               error = 0;

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      if (argc != 3)  {
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
      else if (MFS_alloc_path(&file_name) != MFS_NO_ERROR) {
         fprintf(shell_ptr->STDOUT, "Error, unable to allocate memory for paths\n" );
         return_code = SHELL_EXIT_ERROR;
      }
      else {
         error = _io_rel2abs(file_name,shell_ptr->CURRENT_DIR,(char *) argv[1],PATHNAME_SIZE,shell_ptr->CURRENT_DEVICE_NAME);
         if (error) {
             puts("Error, unable to get path.\n");
             return_code = SHELL_EXIT_ERROR;
             goto err_path1;
         }
         in_fd_1 = open(file_name, O_RDONLY);
         if (0 > in_fd_1) {
             fprintf(shell_ptr->STDOUT, "Error, unable to open file %s\n", argv[1] );
             return_code = SHELL_EXIT_ERROR;
             goto err_path2;
         }

         error = _io_rel2abs(file_name,shell_ptr->CURRENT_DIR,(char *) argv[2],PATHNAME_SIZE,shell_ptr->CURRENT_DEVICE_NAME);
         if(error) {
             puts("Error, unable to get path.\n");
            return_code = SHELL_EXIT_ERROR;
            goto err_path3;
         }
         in_fd_2 = open(file_name, O_RDONLY);
         if (0 > in_fd_2)  {
             fprintf(shell_ptr->STDOUT, "Error, unable to open file %s\n", argv[2] );
             return_code = SHELL_EXIT_ERROR;
             goto err_path4;
         }

         file1 = _mem_alloc_zero(COMPARE_BLOCK_SIZE);
         if (file1 == NULL) {
            fprintf(shell_ptr->STDOUT, "Error, unable to allocate buffer space" );
           return_code = SHELL_EXIT_ERROR;
           goto err_path5;
         }

         file2 = _mem_alloc_zero(COMPARE_BLOCK_SIZE);
         if (file2 == NULL) {
            fprintf(shell_ptr->STDOUT, "Error, unable to allocate buffer space" );
            return_code = SHELL_EXIT_ERROR;
            goto err_path6;
         }

         do {
             size1 = read(in_fd_1, file1, COMPARE_BLOCK_SIZE);
             size2 = read(in_fd_2, file2, COMPARE_BLOCK_SIZE);
             if (size1 != size2) {
                 fprintf(shell_ptr->STDOUT, "Compare failed, files have different sizes\n" );
                 return_code = SHELL_EXIT_ERROR;
                 goto err_path7;
             }
             if (size1 > 0) {
                 if (memcmp(file1, file2, COMPARE_BLOCK_SIZE) != 0) {
                    fprintf(shell_ptr->STDOUT, "Compare failed, files are different\n" );
                    return_code = SHELL_EXIT_ERROR;
                    goto err_path7;
                 }
             }
         } while (size1 > 0);

         fprintf(shell_ptr->STDOUT, "The files are identical\n" );
err_path7:
         _mem_free(file2);
err_path6:
         _mem_free(file1);
err_path5:
err_path4:
         close(in_fd_2);
err_path3:
err_path2:
         close(in_fd_1);
err_path1:
         MFS_free_path(file_name);
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s <file1> <file2> \n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s <file1> <file2>\n", argv[0]);
         fprintf(shell_ptr->STDOUT, "   <file1> = first file to compare\n");
         fprintf(shell_ptr->STDOUT, "   <file2> = second file to compare\n");
      }
   }
   return return_code;
} /* Endbody */

#endif //SHELLCFG_USES_MFS
/* EOF*/
