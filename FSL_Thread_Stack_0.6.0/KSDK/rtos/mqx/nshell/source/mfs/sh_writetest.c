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

#include "nio.h"
#include "fcntl.h"
#include "fs_supp.h"

#if SHELLCFG_USES_MFS
#include <mfs.h>
#include <sh_mfs.h>


extern const char MFS_File_test_data[] ;
extern const uint32_t MFS_File_test_data_size;

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    :   Shell_write_test
* Returned Value   :  int32_t error code
* Comments  :  Reads from a file .
*
*END*---------------------------------------------------------------------*/
#ifndef min
   #define min(a,b) ((a)<(b)?(a):(b))
#endif

int32_t  Shell_write_test(int32_t argc, char *argv[] )
{
   bool           print_usage, shorthelp = FALSE;
   int32_t           return_code = SHELL_EXIT_SUCCESS;
   int fd = -1;
   char              *read_buffer = NULL;
   uint32_t          test_size = MFS_File_test_data_size;
   uint32_t          write_size = 1,num,size,bytes_written;
   SHELL_CONTEXT_PTR shell_ptr = Shell_get_context( argv );
   char              *abs_path = NULL, *filename = NULL;
   int32_t           error = 0;


   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      if (argc != 2) {
         fprintf(shell_ptr->STDOUT, "Error, invalid number of parameters\n");
         return_code = SHELL_EXIT_ERROR;
         print_usage=TRUE;
      } else  {
         read_buffer = _mem_alloc(test_size + 1);
         if (read_buffer==NULL)  {
            fprintf(shell_ptr->STDOUT, "Error, unable to allocate read buffer\n");
            return_code = SHELL_EXIT_ERROR;
         } else {
            _mem_zero (read_buffer, test_size + 1);
            if (MFS_alloc_2paths(&abs_path,&filename) != MFS_NO_ERROR) {
               fprintf(shell_ptr->STDOUT, "Error, unable to allocate memory for paths\n" );
               return_code = SHELL_EXIT_ERROR;
            }
            else
            {
               for (write_size=8;write_size<test_size;write_size=write_size<<1) {
                  fprintf(shell_ptr->STDOUT, "Test Iteration #%d ", write_size);
                  sprintf(filename,"%s_%d",argv[1],write_size);
                  error = _io_rel2abs(abs_path,shell_ptr->CURRENT_DIR,(char *) filename,PATHNAME_SIZE,shell_ptr->CURRENT_DEVICE_NAME);
                  if(!error)
                  {
                     fd = open(abs_path, O_WRONLY);
                  }
                  if (0 > fd) {
                     fprintf(shell_ptr->STDOUT, "Error, unable to open file to write\n");
                     return_code = SHELL_EXIT_ERROR;
                     break;
                  }
                  lseek(fd, 0, SEEK_SET);
                  bytes_written = 0;
                  while ( bytes_written<test_size) {
                     size = min(write_size,test_size-bytes_written);
                     num = write(fd, (void *) &MFS_File_test_data[bytes_written],size);
                     if (num) {
                        bytes_written+=num;
                     } else {
                        fprintf(shell_ptr->STDOUT, "Write failed, offset = %d", bytes_written);
                       break;
                     }
                  }
                  close(fd);
                  fd = open(abs_path, O_RDONLY);
                  if (0 > fd) {
                     fprintf(shell_ptr->STDOUT, "Error, unable to open file to read\n");
                     return_code = SHELL_EXIT_ERROR;
                     break;
                  }
                  if (read(fd, read_buffer, test_size) != test_size) {
                     fprintf(shell_ptr->STDOUT, "Error, unable to read all test data\n");
                     return_code = SHELL_EXIT_ERROR;
                     break;
                  }

                  if (strcmp(MFS_File_test_data,read_buffer)==0) {
                     fprintf(shell_ptr->STDOUT, "Passed\n");
                  } else {
                     fprintf(shell_ptr->STDOUT, "Failed\n");
                     break;
                  }
                  close(fd);
               }
               _mem_free(read_buffer);
               MFS_free_path(abs_path);
               MFS_free_path(filename);
            }
         }
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s <filename> \n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s <filename> \n", argv[0]);
         fprintf(shell_ptr->STDOUT, "   <filename>   = filename to verify\n");
      }
   }
   return return_code;
} /* Endbody */



/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    :   Shell_write_test
* Returned Value   :  int32_t error code
* Comments  :  Reads from a file .
*
*END*---------------------------------------------------------------------*/

int32_t  Shell_append_test(int32_t argc, char *argv[] )
{
   bool           print_usage, shorthelp = FALSE;
   int32_t            return_code = SHELL_EXIT_SUCCESS;
   int fd;
   char              filename[SHELL_MAX_FILELEN];
   char          *read_buffer = NULL;
   uint32_t           test_size = MFS_File_test_data_size;
   uint32_t           write_size = 1,num,size,bytes_written;
   SHELL_CONTEXT_PTR    shell_ptr = Shell_get_context( argv );
   char                 abs_path[SHELL_PATHNAME_LEN];
   int32_t               error = 0;

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      if (argc != 2) {
         fprintf(shell_ptr->STDOUT, "Error, invalid number of parameters\n");
         return_code = SHELL_EXIT_ERROR;
         print_usage=TRUE;
      } else  {
         read_buffer = _mem_alloc(test_size);

         if (read_buffer==NULL)  {
            fprintf(shell_ptr->STDOUT, "Error, unable to allocate read buffer\n");
            return_code = SHELL_EXIT_ERROR;
         } else {
            for (write_size=512;write_size<test_size;write_size=write_size*2) {
               fprintf(shell_ptr->STDOUT, "\nTest Iteration #%d", write_size);
               sprintf(filename,"%s_%d",argv[1],write_size);
               error = _io_rel2abs(abs_path,shell_ptr->CURRENT_DIR,(char *) filename,PATHNAME_SIZE,shell_ptr->CURRENT_DEVICE_NAME);
               bytes_written = 0;
               while ( bytes_written<test_size) {
                  fd = open(abs_path, O_WRONLY);
                  if (0 > fd || error) {
                     fprintf(shell_ptr->STDOUT, "Error, unable to open file to write\n");
                     return_code = SHELL_EXIT_ERROR;
                     break;
                  }
                  lseek(fd, bytes_written, SEEK_SET);
                  size = min(write_size,test_size-bytes_written);
                  num = write(fd, (void *) &MFS_File_test_data[bytes_written],size);
                  close(fd);
                  if (num) {
                     bytes_written+=num;
                  } else {
                     fprintf(shell_ptr->STDOUT, " read failed, offset = %d", bytes_written);
                    break;
                  }
               }
               fd = open(abs_path, O_RDONLY);
               if (0 > fd || error) {
                  fprintf(shell_ptr->STDOUT, "Error, unable to open file to read\n");
                  return_code = SHELL_EXIT_ERROR;
                  break;
               }
               if (read(fd, read_buffer, test_size) != test_size) {
                  fprintf(shell_ptr->STDOUT, "Error, unable to read all test data\n");
                  return_code = SHELL_EXIT_ERROR;
                  break;
               }

               if (strcmp(MFS_File_test_data,read_buffer)==0) {
                  fprintf(shell_ptr->STDOUT, "Passed");
               } else {
                  fprintf(shell_ptr->STDOUT, "Failed");
                  break;
               }
               close(fd);
            }
            _mem_free(read_buffer);
         }
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s <filename> \n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s <filename> \n", argv[0]);
         fprintf(shell_ptr->STDOUT, "   <filename>   = filename to verify\n");
      }
   }
   return return_code;
} /* Endbody */
#endif //SHELLCFG_USES_MFS
