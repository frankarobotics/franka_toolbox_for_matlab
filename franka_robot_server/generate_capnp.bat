@echo off
setlocal

REM Use hard-coded path instead of command line argument
set CAPNP_FILE=interface\rpc.capnp

REM Check if input file exists
if not exist "%CAPNP_FILE%" (
    echo Error: File does not exist: %CAPNP_FILE%
    exit /b 1
)

REM Create output directory if it doesn't exist
if not exist "build" mkdir build
if not exist "build\interface" mkdir build\interface

REM Generate C++ header and source files
capnp compile -oc++:.\build\interface "%CAPNP_FILE%" --src-prefix=interface -I.

REM Rename generated .c++ files to .cpp and then delete the originals
for %%f in (.\build\interface\*.c++) do (
    ren "%%f" "%%~nf.cpp"
    del "%%f" 2>nul
)

echo Cap'n Proto files generated!