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
*   This file contains the type command for shell.
*
*
*END************************************************************************/

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <mqx.h>

#include <shell.h>
#include <sh_prv.h>

#include "nio.h"
#include "fcntl.h"
#include "fs_supp.h"

#if SHELLCFG_USES_MFS
#include <mfs.h>


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name :  Shell_type
*  Returned Value:  none
*  Comments  :  SHELL utility to Ping a host
*
*END*-----------------------------------------------------------------*/

int32_t  Shell_type(int32_t argc, char *argv[] )
{ /* Body */
   bool              print_usage, shorthelp = FALSE;
   int32_t               return_code = SHELL_EXIT_SUCCESS;
   int fd = -1;
   char             *abs_path;
   char c;
   SHELL_CONTEXT_PTR    shell_ptr = Shell_get_context( argv );
   int32_t               error = 0;
   int res;

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      if (argc == 2)  {
         /* check if filesystem is mounted */
         if (0 > Shell_get_current_filesystem(argv))    {
            fprintf(shell_ptr->STDOUT, "Error, file system not mounted\n" );
            return_code = SHELL_EXIT_ERROR;
         }
         else if (MFS_alloc_path(&abs_path) != MFS_NO_ERROR) {
            fprintf(shell_ptr->STDOUT, "Error, unable to allocate memory for paths\n" );
            return_code = SHELL_EXIT_ERROR;
         }
         else {
            error = _io_rel2abs(abs_path,shell_ptr->CURRENT_DIR,(char *) argv[1],PATHNAME_SIZE,shell_ptr->CURRENT_DEVICE_NAME);

            if (error)  {
               fprintf(shell_ptr->STDOUT, "Error, unable to locate file %s.\n", argv[1] );
               return_code = SHELL_EXIT_ERROR;
            } else  {
               fd = open(abs_path, O_RDONLY);
               if (0 <= fd) {
                  do {
//                   c = fgetc(fd);
                     res = read(fd, &c, 1);

                     if (0 < res) {
//                      fputc((char)c, stdout);
                        fputc(c, shell_ptr->STDOUT);
                     }
                  } while (0 < res);
                  close(fd);
                  fprintf(shell_ptr->STDOUT, "\n");
               }
               else {
                  fprintf(shell_ptr->STDOUT, "Error, unable to open file %s.\n", argv[1] );
                  return_code = SHELL_EXIT_ERROR;
               }
            }

            MFS_free_path(abs_path);
         }
      } else  {
         fprintf(shell_ptr->STDOUT, "Error, %s invoked with incorrect number of arguments\n", argv[0]);
         return_code = SHELL_EXIT_ERROR;
         print_usage = TRUE;
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s <filename>\n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s <filename>\n", argv[0]);
         fprintf(shell_ptr->STDOUT, "   <filename>   = filename to display\n");
      }
   }
   return return_code;
} /* Endbody */

#endif //SHELLCFG_USES_MFS

/* EOF */
