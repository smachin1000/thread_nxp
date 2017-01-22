@echo off

rem convert path to backslash format
set ROOTDIR=%1
set ROOTDIR=%ROOTDIR:/=\%
set ROOTDIR=%ROOTDIR:"=%
set OUTPUTDIR=%2
set OUTPUTDIR=%OUTPUTDIR:/=\%
set OUTPUTDIR=%OUTPUTDIR:"=%
set TOOL=%3

rem process one of label bellow
goto label_%TOOL%

:label_armgcc
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
IF NOT EXIST "%OUTPUTDIR%\../config" mkdir "%OUTPUTDIR%\../config"
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\gcc_arm\asm_mac.h" "%OUTPUTDIR%\asm_mac.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\gcc_arm\comp.h" "%OUTPUTDIR%\comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp.h" "%OUTPUTDIR%\psp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_abi.h" "%OUTPUTDIR%\psp_abi.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_comp.h" "%OUTPUTDIR%\psp_comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_math.h" "%OUTPUTDIR%\psp_math.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_prv.h" "%OUTPUTDIR%\psp_prv.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_rev.h" "%OUTPUTDIR%\psp_rev.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_supp.h" "%OUTPUTDIR%\psp_supp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_time.h" "%OUTPUTDIR%\psp_time.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psptypes.h" "%OUTPUTDIR%\psptypes.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpu.h" "%OUTPUTDIR%\psp_cpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpudef.h" "%OUTPUTDIR%\psp_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cortex.h" "%OUTPUTDIR%\cortex.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis_mpu.h" "%OUTPUTDIR%\kinetis_mpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\nvic.h" "%OUTPUTDIR%\nvic.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis.h" "%OUTPUTDIR%\kinetis.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cpu\kinetis_fcan.h" "%OUTPUTDIR%\kinetis_fcan.h" /Y
copy "%ROOTDIR%\config\mcu\mk21d50m\mqx_sdk_config.h" "%OUTPUTDIR%\..\config\mqx_sdk_config.h" /Y
copy "%ROOTDIR%\config\board\twrk21d50m\user_config.h" "%OUTPUTDIR%\..\config\user_config.h" /Y
copy "%ROOTDIR%\config\common\small_ram_config.h" "%OUTPUTDIR%\small_ram_config.h" /Y
copy "%ROOTDIR%\config\common\lite_config.h" "%OUTPUTDIR%\lite_config.h" /Y
copy "%ROOTDIR%\config\common\maximum_config.h" "%OUTPUTDIR%\maximum_config.h" /Y
copy "%ROOTDIR%\config\common\verif_enabled_config.h" "%OUTPUTDIR%\verif_enabled_config.h" /Y
copy "%ROOTDIR%\mqx\source\include\gen_rev.h" "%OUTPUTDIR%\gen_rev.h" /Y
copy "%ROOTDIR%\mqx\source\include\eds.h" "%OUTPUTDIR%\eds.h" /Y
copy "%ROOTDIR%\mqx\source\include\edserial.h" "%OUTPUTDIR%\edserial.h" /Y
copy "%ROOTDIR%\mqx\source\include\event.h" "%OUTPUTDIR%\event.h" /Y
copy "%ROOTDIR%\mqx\source\include\charq.h" "%OUTPUTDIR%\charq.h" /Y
copy "%ROOTDIR%\mqx\source\include\klog.h" "%OUTPUTDIR%\klog.h" /Y
copy "%ROOTDIR%\mqx\source\include\log.h" "%OUTPUTDIR%\log.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwsem.h" "%OUTPUTDIR%\lwsem.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwevent.h" "%OUTPUTDIR%\lwevent.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwlog.h" "%OUTPUTDIR%\lwlog.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmem.h" "%OUTPUTDIR%\lwmem.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmsgq.h" "%OUTPUTDIR%\lwmsgq.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwtimer.h" "%OUTPUTDIR%\lwtimer.h" /Y
copy "%ROOTDIR%\mqx\source\include\message.h" "%OUTPUTDIR%\message.h" /Y
copy "%ROOTDIR%\mqx\source\include\mmu.h" "%OUTPUTDIR%\mmu.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx.h" "%OUTPUTDIR%\mqx.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cnfg.h" "%OUTPUTDIR%\mqx_cnfg.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cpudef.h" "%OUTPUTDIR%\mqx_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_inc.h" "%OUTPUTDIR%\mqx_inc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_ioc.h" "%OUTPUTDIR%\mqx_ioc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_str.h" "%OUTPUTDIR%\mqx_str.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_macros.h" "%OUTPUTDIR%\mqx_macros.h" /Y
copy "%ROOTDIR%\mqx\source\include\mutex.h" "%OUTPUTDIR%\mutex.h" /Y
copy "%ROOTDIR%\mqx\source\include\name.h" "%OUTPUTDIR%\name.h" /Y
copy "%ROOTDIR%\mqx\source\include\partition.h" "%OUTPUTDIR%\partition.h" /Y
copy "%ROOTDIR%\mqx\source\include\pcb.h" "%OUTPUTDIR%\pcb.h" /Y
copy "%ROOTDIR%\mqx\source\include\psptypes_legacy.h" "%OUTPUTDIR%\psptypes_legacy.h" /Y
copy "%ROOTDIR%\mqx\source\include\queue.h" "%OUTPUTDIR%\queue.h" /Y
copy "%ROOTDIR%\mqx\source\include\sem.h" "%OUTPUTDIR%\sem.h" /Y
copy "%ROOTDIR%\mqx\source\include\timer.h" "%OUTPUTDIR%\timer.h" /Y
copy "%ROOTDIR%\mqx\source\include\watchdog.h" "%OUTPUTDIR%\watchdog.h" /Y
copy "%ROOTDIR%\mqx\source\include\unicode.h" "%OUTPUTDIR%\unicode.h" /Y
copy "%ROOTDIR%\mqx\source\include\task.h" "%OUTPUTDIR%\task.h" /Y
copy "%ROOTDIR%\mqx\source\nio\nio.h" "%OUTPUTDIR%\nio.h" /Y
copy "%ROOTDIR%\mqx\source\nio\errno.h" "%OUTPUTDIR%\errno.h" /Y
copy "%ROOTDIR%\mqx\source\nio\ioctl.h" "%OUTPUTDIR%\ioctl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fcntl.h" "%OUTPUTDIR%\fcntl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fs\fs_supp.h" "%OUTPUTDIR%\fs_supp.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_dummy\nio_dummy.h" "%OUTPUTDIR%\nio_dummy.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_mem\nio_mem.h" "%OUTPUTDIR%\nio_mem.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_null\nio_null.h" "%OUTPUTDIR%\nio_null.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_pipe\nio_pipe.h" "%OUTPUTDIR%\nio_pipe.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_serial\nio_serial.h" "%OUTPUTDIR%\nio_serial.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tfs\nio_tfs.h" "%OUTPUTDIR%\nio_tfs.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tty\nio_tty.h" "%OUTPUTDIR%\nio_tty.h" /Y
copy "%ROOTDIR%\mqx\source\tad\tad.h" "%OUTPUTDIR%\tad.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
goto end_script


:label_iar
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
IF NOT EXIST "%OUTPUTDIR%\../config" mkdir "%OUTPUTDIR%\../config"
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\iar\asm_mac.h" "%OUTPUTDIR%\asm_mac.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\iar\comp.h" "%OUTPUTDIR%\comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp.h" "%OUTPUTDIR%\psp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_abi.h" "%OUTPUTDIR%\psp_abi.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_comp.h" "%OUTPUTDIR%\psp_comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_math.h" "%OUTPUTDIR%\psp_math.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_prv.h" "%OUTPUTDIR%\psp_prv.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_rev.h" "%OUTPUTDIR%\psp_rev.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_supp.h" "%OUTPUTDIR%\psp_supp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_time.h" "%OUTPUTDIR%\psp_time.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psptypes.h" "%OUTPUTDIR%\psptypes.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpu.h" "%OUTPUTDIR%\psp_cpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpudef.h" "%OUTPUTDIR%\psp_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cortex.h" "%OUTPUTDIR%\cortex.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis_mpu.h" "%OUTPUTDIR%\kinetis_mpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\nvic.h" "%OUTPUTDIR%\nvic.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis.h" "%OUTPUTDIR%\kinetis.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cpu\kinetis_fcan.h" "%OUTPUTDIR%\kinetis_fcan.h" /Y
copy "%ROOTDIR%\config\mcu\mk21d50m\mqx_sdk_config.h" "%OUTPUTDIR%\..\config\mqx_sdk_config.h" /Y
copy "%ROOTDIR%\config\board\twrk21d50m\user_config.h" "%OUTPUTDIR%\..\config\user_config.h" /Y
copy "%ROOTDIR%\config\common\small_ram_config.h" "%OUTPUTDIR%\small_ram_config.h" /Y
copy "%ROOTDIR%\config\common\lite_config.h" "%OUTPUTDIR%\lite_config.h" /Y
copy "%ROOTDIR%\config\common\maximum_config.h" "%OUTPUTDIR%\maximum_config.h" /Y
copy "%ROOTDIR%\config\common\verif_enabled_config.h" "%OUTPUTDIR%\verif_enabled_config.h" /Y
copy "%ROOTDIR%\mqx\source\include\gen_rev.h" "%OUTPUTDIR%\gen_rev.h" /Y
copy "%ROOTDIR%\mqx\source\include\eds.h" "%OUTPUTDIR%\eds.h" /Y
copy "%ROOTDIR%\mqx\source\include\edserial.h" "%OUTPUTDIR%\edserial.h" /Y
copy "%ROOTDIR%\mqx\source\include\event.h" "%OUTPUTDIR%\event.h" /Y
copy "%ROOTDIR%\mqx\source\include\charq.h" "%OUTPUTDIR%\charq.h" /Y
copy "%ROOTDIR%\mqx\source\include\klog.h" "%OUTPUTDIR%\klog.h" /Y
copy "%ROOTDIR%\mqx\source\include\log.h" "%OUTPUTDIR%\log.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwsem.h" "%OUTPUTDIR%\lwsem.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwevent.h" "%OUTPUTDIR%\lwevent.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwlog.h" "%OUTPUTDIR%\lwlog.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmem.h" "%OUTPUTDIR%\lwmem.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmsgq.h" "%OUTPUTDIR%\lwmsgq.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwtimer.h" "%OUTPUTDIR%\lwtimer.h" /Y
copy "%ROOTDIR%\mqx\source\include\message.h" "%OUTPUTDIR%\message.h" /Y
copy "%ROOTDIR%\mqx\source\include\mmu.h" "%OUTPUTDIR%\mmu.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx.h" "%OUTPUTDIR%\mqx.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cnfg.h" "%OUTPUTDIR%\mqx_cnfg.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cpudef.h" "%OUTPUTDIR%\mqx_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_inc.h" "%OUTPUTDIR%\mqx_inc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_ioc.h" "%OUTPUTDIR%\mqx_ioc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_str.h" "%OUTPUTDIR%\mqx_str.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_macros.h" "%OUTPUTDIR%\mqx_macros.h" /Y
copy "%ROOTDIR%\mqx\source\include\mutex.h" "%OUTPUTDIR%\mutex.h" /Y
copy "%ROOTDIR%\mqx\source\include\name.h" "%OUTPUTDIR%\name.h" /Y
copy "%ROOTDIR%\mqx\source\include\partition.h" "%OUTPUTDIR%\partition.h" /Y
copy "%ROOTDIR%\mqx\source\include\pcb.h" "%OUTPUTDIR%\pcb.h" /Y
copy "%ROOTDIR%\mqx\source\include\psptypes_legacy.h" "%OUTPUTDIR%\psptypes_legacy.h" /Y
copy "%ROOTDIR%\mqx\source\include\queue.h" "%OUTPUTDIR%\queue.h" /Y
copy "%ROOTDIR%\mqx\source\include\sem.h" "%OUTPUTDIR%\sem.h" /Y
copy "%ROOTDIR%\mqx\source\include\timer.h" "%OUTPUTDIR%\timer.h" /Y
copy "%ROOTDIR%\mqx\source\include\watchdog.h" "%OUTPUTDIR%\watchdog.h" /Y
copy "%ROOTDIR%\mqx\source\include\unicode.h" "%OUTPUTDIR%\unicode.h" /Y
copy "%ROOTDIR%\mqx\source\include\task.h" "%OUTPUTDIR%\task.h" /Y
copy "%ROOTDIR%\mqx\source\nio\nio.h" "%OUTPUTDIR%\nio.h" /Y
copy "%ROOTDIR%\mqx\source\nio\errno.h" "%OUTPUTDIR%\errno.h" /Y
copy "%ROOTDIR%\mqx\source\nio\ioctl.h" "%OUTPUTDIR%\ioctl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fcntl.h" "%OUTPUTDIR%\fcntl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fs\fs_supp.h" "%OUTPUTDIR%\fs_supp.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_dummy\nio_dummy.h" "%OUTPUTDIR%\nio_dummy.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_mem\nio_mem.h" "%OUTPUTDIR%\nio_mem.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_null\nio_null.h" "%OUTPUTDIR%\nio_null.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_pipe\nio_pipe.h" "%OUTPUTDIR%\nio_pipe.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_serial\nio_serial.h" "%OUTPUTDIR%\nio_serial.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tfs\nio_tfs.h" "%OUTPUTDIR%\nio_tfs.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tty\nio_tty.h" "%OUTPUTDIR%\nio_tty.h" /Y
copy "%ROOTDIR%\mqx\source\tad\tad.h" "%OUTPUTDIR%\tad.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
goto end_script


:label_kds
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
IF NOT EXIST "%OUTPUTDIR%\../config" mkdir "%OUTPUTDIR%\../config"
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\gcc_arm\asm_mac.h" "%OUTPUTDIR%\asm_mac.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\gcc_arm\comp.h" "%OUTPUTDIR%\comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp.h" "%OUTPUTDIR%\psp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_abi.h" "%OUTPUTDIR%\psp_abi.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_comp.h" "%OUTPUTDIR%\psp_comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_math.h" "%OUTPUTDIR%\psp_math.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_prv.h" "%OUTPUTDIR%\psp_prv.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_rev.h" "%OUTPUTDIR%\psp_rev.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_supp.h" "%OUTPUTDIR%\psp_supp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_time.h" "%OUTPUTDIR%\psp_time.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psptypes.h" "%OUTPUTDIR%\psptypes.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpu.h" "%OUTPUTDIR%\psp_cpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpudef.h" "%OUTPUTDIR%\psp_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cortex.h" "%OUTPUTDIR%\cortex.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis_mpu.h" "%OUTPUTDIR%\kinetis_mpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\nvic.h" "%OUTPUTDIR%\nvic.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis.h" "%OUTPUTDIR%\kinetis.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cpu\kinetis_fcan.h" "%OUTPUTDIR%\kinetis_fcan.h" /Y
copy "%ROOTDIR%\config\mcu\mk21d50m\mqx_sdk_config.h" "%OUTPUTDIR%\..\config\mqx_sdk_config.h" /Y
copy "%ROOTDIR%\config\board\twrk21d50m\user_config.h" "%OUTPUTDIR%\..\config\user_config.h" /Y
copy "%ROOTDIR%\config\common\small_ram_config.h" "%OUTPUTDIR%\small_ram_config.h" /Y
copy "%ROOTDIR%\config\common\lite_config.h" "%OUTPUTDIR%\lite_config.h" /Y
copy "%ROOTDIR%\config\common\maximum_config.h" "%OUTPUTDIR%\maximum_config.h" /Y
copy "%ROOTDIR%\config\common\verif_enabled_config.h" "%OUTPUTDIR%\verif_enabled_config.h" /Y
copy "%ROOTDIR%\mqx\source\include\gen_rev.h" "%OUTPUTDIR%\gen_rev.h" /Y
copy "%ROOTDIR%\mqx\source\include\eds.h" "%OUTPUTDIR%\eds.h" /Y
copy "%ROOTDIR%\mqx\source\include\edserial.h" "%OUTPUTDIR%\edserial.h" /Y
copy "%ROOTDIR%\mqx\source\include\event.h" "%OUTPUTDIR%\event.h" /Y
copy "%ROOTDIR%\mqx\source\include\charq.h" "%OUTPUTDIR%\charq.h" /Y
copy "%ROOTDIR%\mqx\source\include\klog.h" "%OUTPUTDIR%\klog.h" /Y
copy "%ROOTDIR%\mqx\source\include\log.h" "%OUTPUTDIR%\log.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwsem.h" "%OUTPUTDIR%\lwsem.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwevent.h" "%OUTPUTDIR%\lwevent.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwlog.h" "%OUTPUTDIR%\lwlog.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmem.h" "%OUTPUTDIR%\lwmem.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmsgq.h" "%OUTPUTDIR%\lwmsgq.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwtimer.h" "%OUTPUTDIR%\lwtimer.h" /Y
copy "%ROOTDIR%\mqx\source\include\message.h" "%OUTPUTDIR%\message.h" /Y
copy "%ROOTDIR%\mqx\source\include\mmu.h" "%OUTPUTDIR%\mmu.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx.h" "%OUTPUTDIR%\mqx.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cnfg.h" "%OUTPUTDIR%\mqx_cnfg.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cpudef.h" "%OUTPUTDIR%\mqx_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_inc.h" "%OUTPUTDIR%\mqx_inc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_ioc.h" "%OUTPUTDIR%\mqx_ioc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_str.h" "%OUTPUTDIR%\mqx_str.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_macros.h" "%OUTPUTDIR%\mqx_macros.h" /Y
copy "%ROOTDIR%\mqx\source\include\mutex.h" "%OUTPUTDIR%\mutex.h" /Y
copy "%ROOTDIR%\mqx\source\include\name.h" "%OUTPUTDIR%\name.h" /Y
copy "%ROOTDIR%\mqx\source\include\partition.h" "%OUTPUTDIR%\partition.h" /Y
copy "%ROOTDIR%\mqx\source\include\pcb.h" "%OUTPUTDIR%\pcb.h" /Y
copy "%ROOTDIR%\mqx\source\include\psptypes_legacy.h" "%OUTPUTDIR%\psptypes_legacy.h" /Y
copy "%ROOTDIR%\mqx\source\include\queue.h" "%OUTPUTDIR%\queue.h" /Y
copy "%ROOTDIR%\mqx\source\include\sem.h" "%OUTPUTDIR%\sem.h" /Y
copy "%ROOTDIR%\mqx\source\include\timer.h" "%OUTPUTDIR%\timer.h" /Y
copy "%ROOTDIR%\mqx\source\include\watchdog.h" "%OUTPUTDIR%\watchdog.h" /Y
copy "%ROOTDIR%\mqx\source\include\unicode.h" "%OUTPUTDIR%\unicode.h" /Y
copy "%ROOTDIR%\mqx\source\include\task.h" "%OUTPUTDIR%\task.h" /Y
copy "%ROOTDIR%\mqx\source\nio\nio.h" "%OUTPUTDIR%\nio.h" /Y
copy "%ROOTDIR%\mqx\source\nio\errno.h" "%OUTPUTDIR%\errno.h" /Y
copy "%ROOTDIR%\mqx\source\nio\ioctl.h" "%OUTPUTDIR%\ioctl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fcntl.h" "%OUTPUTDIR%\fcntl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fs\fs_supp.h" "%OUTPUTDIR%\fs_supp.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_dummy\nio_dummy.h" "%OUTPUTDIR%\nio_dummy.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_mem\nio_mem.h" "%OUTPUTDIR%\nio_mem.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_null\nio_null.h" "%OUTPUTDIR%\nio_null.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_pipe\nio_pipe.h" "%OUTPUTDIR%\nio_pipe.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_serial\nio_serial.h" "%OUTPUTDIR%\nio_serial.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tfs\nio_tfs.h" "%OUTPUTDIR%\nio_tfs.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tty\nio_tty.h" "%OUTPUTDIR%\nio_tty.h" /Y
copy "%ROOTDIR%\mqx\source\tad\tad.h" "%OUTPUTDIR%\tad.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
goto end_script


:label_atl
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
IF NOT EXIST "%OUTPUTDIR%\../config" mkdir "%OUTPUTDIR%\../config"
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\gcc_arm\asm_mac.h" "%OUTPUTDIR%\asm_mac.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\gcc_arm\comp.h" "%OUTPUTDIR%\comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp.h" "%OUTPUTDIR%\psp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_abi.h" "%OUTPUTDIR%\psp_abi.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_comp.h" "%OUTPUTDIR%\psp_comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_math.h" "%OUTPUTDIR%\psp_math.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_prv.h" "%OUTPUTDIR%\psp_prv.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_rev.h" "%OUTPUTDIR%\psp_rev.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_supp.h" "%OUTPUTDIR%\psp_supp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_time.h" "%OUTPUTDIR%\psp_time.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psptypes.h" "%OUTPUTDIR%\psptypes.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpu.h" "%OUTPUTDIR%\psp_cpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpudef.h" "%OUTPUTDIR%\psp_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cortex.h" "%OUTPUTDIR%\cortex.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis_mpu.h" "%OUTPUTDIR%\kinetis_mpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\nvic.h" "%OUTPUTDIR%\nvic.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis.h" "%OUTPUTDIR%\kinetis.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cpu\kinetis_fcan.h" "%OUTPUTDIR%\kinetis_fcan.h" /Y
copy "%ROOTDIR%\config\mcu\mk21d50m\mqx_sdk_config.h" "%OUTPUTDIR%\..\config\mqx_sdk_config.h" /Y
copy "%ROOTDIR%\config\board\twrk21d50m\user_config.h" "%OUTPUTDIR%\..\config\user_config.h" /Y
copy "%ROOTDIR%\config\common\small_ram_config.h" "%OUTPUTDIR%\small_ram_config.h" /Y
copy "%ROOTDIR%\config\common\lite_config.h" "%OUTPUTDIR%\lite_config.h" /Y
copy "%ROOTDIR%\config\common\maximum_config.h" "%OUTPUTDIR%\maximum_config.h" /Y
copy "%ROOTDIR%\config\common\verif_enabled_config.h" "%OUTPUTDIR%\verif_enabled_config.h" /Y
copy "%ROOTDIR%\mqx\source\include\gen_rev.h" "%OUTPUTDIR%\gen_rev.h" /Y
copy "%ROOTDIR%\mqx\source\include\eds.h" "%OUTPUTDIR%\eds.h" /Y
copy "%ROOTDIR%\mqx\source\include\edserial.h" "%OUTPUTDIR%\edserial.h" /Y
copy "%ROOTDIR%\mqx\source\include\event.h" "%OUTPUTDIR%\event.h" /Y
copy "%ROOTDIR%\mqx\source\include\charq.h" "%OUTPUTDIR%\charq.h" /Y
copy "%ROOTDIR%\mqx\source\include\klog.h" "%OUTPUTDIR%\klog.h" /Y
copy "%ROOTDIR%\mqx\source\include\log.h" "%OUTPUTDIR%\log.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwsem.h" "%OUTPUTDIR%\lwsem.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwevent.h" "%OUTPUTDIR%\lwevent.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwlog.h" "%OUTPUTDIR%\lwlog.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmem.h" "%OUTPUTDIR%\lwmem.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmsgq.h" "%OUTPUTDIR%\lwmsgq.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwtimer.h" "%OUTPUTDIR%\lwtimer.h" /Y
copy "%ROOTDIR%\mqx\source\include\message.h" "%OUTPUTDIR%\message.h" /Y
copy "%ROOTDIR%\mqx\source\include\mmu.h" "%OUTPUTDIR%\mmu.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx.h" "%OUTPUTDIR%\mqx.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cnfg.h" "%OUTPUTDIR%\mqx_cnfg.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cpudef.h" "%OUTPUTDIR%\mqx_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_inc.h" "%OUTPUTDIR%\mqx_inc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_ioc.h" "%OUTPUTDIR%\mqx_ioc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_str.h" "%OUTPUTDIR%\mqx_str.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_macros.h" "%OUTPUTDIR%\mqx_macros.h" /Y
copy "%ROOTDIR%\mqx\source\include\mutex.h" "%OUTPUTDIR%\mutex.h" /Y
copy "%ROOTDIR%\mqx\source\include\name.h" "%OUTPUTDIR%\name.h" /Y
copy "%ROOTDIR%\mqx\source\include\partition.h" "%OUTPUTDIR%\partition.h" /Y
copy "%ROOTDIR%\mqx\source\include\pcb.h" "%OUTPUTDIR%\pcb.h" /Y
copy "%ROOTDIR%\mqx\source\include\psptypes_legacy.h" "%OUTPUTDIR%\psptypes_legacy.h" /Y
copy "%ROOTDIR%\mqx\source\include\queue.h" "%OUTPUTDIR%\queue.h" /Y
copy "%ROOTDIR%\mqx\source\include\sem.h" "%OUTPUTDIR%\sem.h" /Y
copy "%ROOTDIR%\mqx\source\include\timer.h" "%OUTPUTDIR%\timer.h" /Y
copy "%ROOTDIR%\mqx\source\include\watchdog.h" "%OUTPUTDIR%\watchdog.h" /Y
copy "%ROOTDIR%\mqx\source\include\unicode.h" "%OUTPUTDIR%\unicode.h" /Y
copy "%ROOTDIR%\mqx\source\include\task.h" "%OUTPUTDIR%\task.h" /Y
copy "%ROOTDIR%\mqx\source\nio\nio.h" "%OUTPUTDIR%\nio.h" /Y
copy "%ROOTDIR%\mqx\source\nio\errno.h" "%OUTPUTDIR%\errno.h" /Y
copy "%ROOTDIR%\mqx\source\nio\ioctl.h" "%OUTPUTDIR%\ioctl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fcntl.h" "%OUTPUTDIR%\fcntl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fs\fs_supp.h" "%OUTPUTDIR%\fs_supp.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_dummy\nio_dummy.h" "%OUTPUTDIR%\nio_dummy.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_mem\nio_mem.h" "%OUTPUTDIR%\nio_mem.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_null\nio_null.h" "%OUTPUTDIR%\nio_null.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_pipe\nio_pipe.h" "%OUTPUTDIR%\nio_pipe.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_serial\nio_serial.h" "%OUTPUTDIR%\nio_serial.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tfs\nio_tfs.h" "%OUTPUTDIR%\nio_tfs.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tty\nio_tty.h" "%OUTPUTDIR%\nio_tty.h" /Y
copy "%ROOTDIR%\mqx\source\tad\tad.h" "%OUTPUTDIR%\tad.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
goto end_script


:label_mdk
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
IF NOT EXIST "%OUTPUTDIR%\../config" mkdir "%OUTPUTDIR%\../config"
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\rv_mdk\asm_mac.h" "%OUTPUTDIR%\asm_mac.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\compiler\rv_mdk\comp.h" "%OUTPUTDIR%\comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp.h" "%OUTPUTDIR%\psp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_abi.h" "%OUTPUTDIR%\psp_abi.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_comp.h" "%OUTPUTDIR%\psp_comp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_math.h" "%OUTPUTDIR%\psp_math.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_prv.h" "%OUTPUTDIR%\psp_prv.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_rev.h" "%OUTPUTDIR%\psp_rev.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_supp.h" "%OUTPUTDIR%\psp_supp.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_time.h" "%OUTPUTDIR%\psp_time.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psptypes.h" "%OUTPUTDIR%\psptypes.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpu.h" "%OUTPUTDIR%\psp_cpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\psp_cpudef.h" "%OUTPUTDIR%\psp_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cortex.h" "%OUTPUTDIR%\cortex.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis_mpu.h" "%OUTPUTDIR%\kinetis_mpu.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\nvic.h" "%OUTPUTDIR%\nvic.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\kinetis.h" "%OUTPUTDIR%\kinetis.h" /Y
copy "%ROOTDIR%\mqx\source\psp\cortex_m\cpu\kinetis_fcan.h" "%OUTPUTDIR%\kinetis_fcan.h" /Y
copy "%ROOTDIR%\config\mcu\mk21d50m\mqx_sdk_config.h" "%OUTPUTDIR%\..\config\mqx_sdk_config.h" /Y
copy "%ROOTDIR%\config\board\twrk21d50m\user_config.h" "%OUTPUTDIR%\..\config\user_config.h" /Y
copy "%ROOTDIR%\config\common\small_ram_config.h" "%OUTPUTDIR%\small_ram_config.h" /Y
copy "%ROOTDIR%\config\common\lite_config.h" "%OUTPUTDIR%\lite_config.h" /Y
copy "%ROOTDIR%\config\common\maximum_config.h" "%OUTPUTDIR%\maximum_config.h" /Y
copy "%ROOTDIR%\config\common\verif_enabled_config.h" "%OUTPUTDIR%\verif_enabled_config.h" /Y
copy "%ROOTDIR%\mqx\source\include\gen_rev.h" "%OUTPUTDIR%\gen_rev.h" /Y
copy "%ROOTDIR%\mqx\source\include\eds.h" "%OUTPUTDIR%\eds.h" /Y
copy "%ROOTDIR%\mqx\source\include\edserial.h" "%OUTPUTDIR%\edserial.h" /Y
copy "%ROOTDIR%\mqx\source\include\event.h" "%OUTPUTDIR%\event.h" /Y
copy "%ROOTDIR%\mqx\source\include\charq.h" "%OUTPUTDIR%\charq.h" /Y
copy "%ROOTDIR%\mqx\source\include\klog.h" "%OUTPUTDIR%\klog.h" /Y
copy "%ROOTDIR%\mqx\source\include\log.h" "%OUTPUTDIR%\log.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwsem.h" "%OUTPUTDIR%\lwsem.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwevent.h" "%OUTPUTDIR%\lwevent.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwlog.h" "%OUTPUTDIR%\lwlog.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmem.h" "%OUTPUTDIR%\lwmem.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwmsgq.h" "%OUTPUTDIR%\lwmsgq.h" /Y
copy "%ROOTDIR%\mqx\source\include\lwtimer.h" "%OUTPUTDIR%\lwtimer.h" /Y
copy "%ROOTDIR%\mqx\source\include\message.h" "%OUTPUTDIR%\message.h" /Y
copy "%ROOTDIR%\mqx\source\include\mmu.h" "%OUTPUTDIR%\mmu.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx.h" "%OUTPUTDIR%\mqx.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cnfg.h" "%OUTPUTDIR%\mqx_cnfg.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_cpudef.h" "%OUTPUTDIR%\mqx_cpudef.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_inc.h" "%OUTPUTDIR%\mqx_inc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_ioc.h" "%OUTPUTDIR%\mqx_ioc.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_str.h" "%OUTPUTDIR%\mqx_str.h" /Y
copy "%ROOTDIR%\mqx\source\include\mqx_macros.h" "%OUTPUTDIR%\mqx_macros.h" /Y
copy "%ROOTDIR%\mqx\source\include\mutex.h" "%OUTPUTDIR%\mutex.h" /Y
copy "%ROOTDIR%\mqx\source\include\name.h" "%OUTPUTDIR%\name.h" /Y
copy "%ROOTDIR%\mqx\source\include\partition.h" "%OUTPUTDIR%\partition.h" /Y
copy "%ROOTDIR%\mqx\source\include\pcb.h" "%OUTPUTDIR%\pcb.h" /Y
copy "%ROOTDIR%\mqx\source\include\psptypes_legacy.h" "%OUTPUTDIR%\psptypes_legacy.h" /Y
copy "%ROOTDIR%\mqx\source\include\queue.h" "%OUTPUTDIR%\queue.h" /Y
copy "%ROOTDIR%\mqx\source\include\sem.h" "%OUTPUTDIR%\sem.h" /Y
copy "%ROOTDIR%\mqx\source\include\timer.h" "%OUTPUTDIR%\timer.h" /Y
copy "%ROOTDIR%\mqx\source\include\watchdog.h" "%OUTPUTDIR%\watchdog.h" /Y
copy "%ROOTDIR%\mqx\source\include\unicode.h" "%OUTPUTDIR%\unicode.h" /Y
copy "%ROOTDIR%\mqx\source\include\task.h" "%OUTPUTDIR%\task.h" /Y
copy "%ROOTDIR%\mqx\source\nio\nio.h" "%OUTPUTDIR%\nio.h" /Y
copy "%ROOTDIR%\mqx\source\nio\errno.h" "%OUTPUTDIR%\errno.h" /Y
copy "%ROOTDIR%\mqx\source\nio\ioctl.h" "%OUTPUTDIR%\ioctl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fcntl.h" "%OUTPUTDIR%\fcntl.h" /Y
copy "%ROOTDIR%\mqx\source\nio\fs\fs_supp.h" "%OUTPUTDIR%\fs_supp.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_dummy\nio_dummy.h" "%OUTPUTDIR%\nio_dummy.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_mem\nio_mem.h" "%OUTPUTDIR%\nio_mem.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_null\nio_null.h" "%OUTPUTDIR%\nio_null.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_pipe\nio_pipe.h" "%OUTPUTDIR%\nio_pipe.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_serial\nio_serial.h" "%OUTPUTDIR%\nio_serial.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tfs\nio_tfs.h" "%OUTPUTDIR%\nio_tfs.h" /Y
copy "%ROOTDIR%\mqx\source\nio\drivers\nio_tty\nio_tty.h" "%OUTPUTDIR%\nio_tty.h" /Y
copy "%ROOTDIR%\mqx\source\tad\tad.h" "%OUTPUTDIR%\tad.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf.h" "%OUTPUTDIR%\tlsf.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsf_adaptation.h" "%OUTPUTDIR%\tlsf_adaptation.h" /Y
copy "%ROOTDIR%\mqx\source\include\tlsfbits.h" "%OUTPUTDIR%\tlsfbits.h" /Y
goto end_script



:end_script

