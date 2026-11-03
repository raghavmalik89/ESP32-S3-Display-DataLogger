@echo off
rem sched_test.bat — build + run the deterministic audio scheduler host test.
rem Uses zig cc (pip install ziglang) because esp-clang ships no x86 backend.
rem Run from the repo root:  tools\win\sched_test.bat
setlocal
python -m ziglang cc -O2 -std=c99 --target=x86_64-windows-gnu ^
    -I main tests\audio_sched_test.c main\vario_core.c ^
    -o tests\audio_sched_test.exe || exit /b 1
tests\audio_sched_test.exe
exit /b %ERRORLEVEL%
