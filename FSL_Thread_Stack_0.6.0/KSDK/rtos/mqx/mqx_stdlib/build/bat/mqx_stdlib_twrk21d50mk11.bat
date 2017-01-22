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
copy "%ROOTDIR%\mqx_stdlib\source\include\std_prv.h" "%OUTPUTDIR%\std_prv.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\stdio.h" "%OUTPUTDIR%\stdio.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\strings.h" "%OUTPUTDIR%\strings.h" /Y
goto end_script


:label_iar
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
copy "%ROOTDIR%\mqx_stdlib\source\include\std_prv.h" "%OUTPUTDIR%\std_prv.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\stdio.h" "%OUTPUTDIR%\stdio.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\strings.h" "%OUTPUTDIR%\strings.h" /Y
goto end_script


:label_kds
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
copy "%ROOTDIR%\mqx_stdlib\source\include\std_prv.h" "%OUTPUTDIR%\std_prv.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\stdio.h" "%OUTPUTDIR%\stdio.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\strings.h" "%OUTPUTDIR%\strings.h" /Y
goto end_script


:label_atl
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
copy "%ROOTDIR%\mqx_stdlib\source\include\std_prv.h" "%OUTPUTDIR%\std_prv.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\stdio.h" "%OUTPUTDIR%\stdio.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\strings.h" "%OUTPUTDIR%\strings.h" /Y
goto end_script


:label_mdk
IF NOT EXIST "%OUTPUTDIR%" mkdir "%OUTPUTDIR%"
IF NOT EXIST "%OUTPUTDIR%\." mkdir "%OUTPUTDIR%\."
copy "%ROOTDIR%\mqx_stdlib\source\include\std_prv.h" "%OUTPUTDIR%\std_prv.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\stdio.h" "%OUTPUTDIR%\stdio.h" /Y
copy "%ROOTDIR%\mqx_stdlib\source\include\strings.h" "%OUTPUTDIR%\strings.h" /Y
goto end_script



:end_script

