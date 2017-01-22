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
*   This file contains the exit command.
*
*
*END************************************************************************/

#include <stdio.h>
#include <mqx.h>
#include <mqx_inc.h> /* internal kernel structures */
#include "shell.h"
#include "sh_prv.h"


/*FUNCTION*-------------------------------------------------------------
*
*  Function Name :  Shell_exit
*  Returned Value:  none
*  Comments  :  SHELL utility to exit a task
*  Usage:
*
*END*-----------------------------------------------------------------*/

int32_t  Shell_exit(int32_t argc, char *argv[] )
{
   bool              print_usage, shorthelp = FALSE;
   int32_t               return_code = SHELL_EXIT_SUCCESS;
   SHELL_CONTEXT_PTR    shell_ptr = Shell_get_context( argv );
   FILE * this_stdin;
   register KERNEL_DATA_STRUCT_PTR  kernel_data;
   register TD_STRUCT_PTR           active_ptr;
   _GET_KERNEL_DATA(kernel_data);
   active_ptr = kernel_data->ACTIVE_PTR;

   print_usage = Shell_check_help_request(argc, argv, &shorthelp );

   if (!print_usage)  {
      shell_ptr = Shell_get_context( argv );
      shell_ptr->EXIT = TRUE;
      if(active_ptr->STDIN_STREAM)
      {
        this_stdin = active_ptr->STDIN_STREAM;
      }
      else
      {
        this_stdin = stdin;
      }
      if (shell_ptr->COMMAND_FP != this_stdin)  {
         fclose(shell_ptr->COMMAND_FP);
      }
   }

   if (print_usage)  {
      if (shorthelp)  {
         fprintf(shell_ptr->STDOUT, "%s\n", argv[0]);
      } else  {
         fprintf(shell_ptr->STDOUT, "Usage: %s\n", argv[0]);
      }
   }
   return return_code;
} /* Endbody */

/* EOF */
