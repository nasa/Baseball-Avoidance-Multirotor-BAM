@echo off
setlocal enableDelayedExpansion

rem Grab the current directory
set TEMP_HOME=%~dp0
set BAM_EXE_DIR=%TEMP_HOME%..\..\AutoCode\BAM_app
pushd %BAM_EXE_DIR%

for /l %%i in (1,1,5) do (

  rem Pad w/ leading zeroes
  set ITER=00%%i
  set ITER=!ITER:~-2!

  rem binary input filename
  set PFILENAME=pFile!ITER!.bin

  rem output filename
  set OUTFILENAME=BAM!ITER!.mat

  rem invoke executable, relative to current directory
  .\BAM_app.exe < !PFILENAME! !OUTFILENAME!

)

popd

pause