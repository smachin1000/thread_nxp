SET CWD=%CD% 
cd %CWD% 
cmake -DCMAKE_TOOLCHAIN_FILE=armgcc.cmake -DCMAKE_BUILD_TYPE="release"  -G "MinGW Makefiles" 
mingw32-make all 
if not "%1" == "nopause" (pause) 
