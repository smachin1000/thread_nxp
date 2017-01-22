INCLUDE(CMakeForceCompiler)

SET(CMAKE_SYSTEM_NAME Generic)

# this file was generated
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(nshell)


SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)


ENABLE_LANGUAGE(ASM)
STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE)


# -----------------------------------------------
# common settings
# -----------------------------------------------

# common asm flags
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m4")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mfloat-abi=soft")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -Xassembler --no-warn")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -std=gnu99")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -Wall")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mthumb")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -g2")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -gdwarf-2")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -gstrict-dwarf")

# common asm defines
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DCPU_MK21DN512AVMC5=1")
SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DFSL_RTOS_MQX=1")

# common c compiler flags
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=soft")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-strict-aliasing")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-missing-braces")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ffunction-sections")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mthumb")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g2")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -gdwarf-2")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -gstrict-dwarf")

# common c compiler defines
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCPU_MK21DN512AVMC5=1")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DFSL_RTOS_MQX=1")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_AEABI_LC_CTYPE=C")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__STRICT_ANSI__=1")


IF(CMAKE_BUILD_TYPE MATCHES "debug")
# -----------------------------------------------
# special settings for 'debug'
# -----------------------------------------------

#   asm defines
    SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -D_DEBUG=1")

#   c compiler flags
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")

#   c compiler defines
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_DEBUG=1")

    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../source/include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m/compiler/gcc_arm")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m/cpu")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/startup")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/CMSIS/Include/device")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/CMSIS/Include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../config/mcu/mk21d50m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../config/board/twrk21d50m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../boards/twrk21d50m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/startup/MK21DA5")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../config/common")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/src")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/fs")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_dummy")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_mem")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_null")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_pipe")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_serial")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_tfs")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_tty")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/tad")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx_stdlib/source/include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx_stdlib/source/stdio")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/hal/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/drivers/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/drivers/src/mpu")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/drivers/src/uart")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/utilities/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../filesystem/mfs/source/mfs_sdcard/nio_esdhc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../..")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/osa/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/system/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m/core/M4")
    SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/twrk21d50m.armgcc/debug/nshell)
ENDIF()
IF(CMAKE_BUILD_TYPE MATCHES "release")
# -----------------------------------------------
# special settings for 'release'
# -----------------------------------------------

#   asm defines
    SET(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -DNDEBUG=1")

#   c compiler flags
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os")

#   c compiler defines
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNDEBUG=1")

    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../source/include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m/compiler/gcc_arm")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m/cpu")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/startup")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/CMSIS/Include/device")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/CMSIS/Include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../config/mcu/mk21d50m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../config/board/twrk21d50m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../boards/twrk21d50m")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/startup/MK21DA5")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../config/common")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/src")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/fs")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_dummy")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_mem")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_null")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_pipe")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_serial")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_tfs")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/nio/drivers/nio_tty")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/tad")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx_stdlib/source/include")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx_stdlib/source/stdio")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/hal/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/drivers/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/drivers/src/mpu")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/drivers/src/uart")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/utilities/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../filesystem/mfs/source/mfs_sdcard/nio_esdhc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../..")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/osa/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../platform/system/inc")
    INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/../../../../mqx/source/psp/cortex_m/core/M4")
    SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/twrk21d50m.armgcc/release/nshell)
ENDIF()


ADD_LIBRARY(lib_nshell.a STATIC
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/mqx/sh_abort.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/mqx/sh_kill.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/mqx/sh_pause.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/mqx/sh_tad.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/include/sh_prv.h"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/include/shell.h"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/shell/sh_cmdlist.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/shell/sh_exit.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/shell/sh_help.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/shell/sh_sh.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/shell/sh_util.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/shell/shell.c"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/include/sh_rtcs.h"
"${CMAKE_CURRENT_SOURCE_DIR}/../../../source/include/sh_enet.h"
)


IF(CMAKE_BUILD_TYPE MATCHES "debug")
    ADD_CUSTOM_COMMAND(TARGET lib_nshell.a POST_BUILD COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../bat/nshell_twrk21d50m.bat ${CMAKE_CURRENT_SOURCE_DIR}/../../../.. ${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/twrk21d50m.armgcc/debug/nshell armgcc)
ENDIF()
IF(CMAKE_BUILD_TYPE MATCHES "release")
    ADD_CUSTOM_COMMAND(TARGET lib_nshell.a POST_BUILD COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../bat/nshell_twrk21d50m.bat ${CMAKE_CURRENT_SOURCE_DIR}/../../../.. ${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/twrk21d50m.armgcc/release/nshell armgcc)
ENDIF()


