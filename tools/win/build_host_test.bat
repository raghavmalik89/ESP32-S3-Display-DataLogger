@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >/dev/null
cd /d C:\Dev\esp32s3-sensor-logger
if "%1"=="double" (set DEF=/DVARIO_USE_DOUBLE) else (set DEF=)
cl /nologo /TC %DEF% /I main /Fetest_filters_host.exe tests\test_filters_host.c main\filters.c main\vario_core.c
