@echo off
rem Minimal ESP-IDF v6.1 environment for this project (bypasses export.bat).
set IDF_PATH=C:\esp\v6.1\esp-idf
set IDF_TOOLS_PATH=C:\Espressif
set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf6.1_py3.14_env
set ESP_IDF_VERSION=6.1
set ESP_ROM_ELF_DIR=C:\Espressif\tools\esp-rom-elfs\20241011
set PATH=C:\Espressif\python_env\idf6.1_py3.14_env\Scripts;C:\Espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin;C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin;C:\Espressif\tools\cmake\4.0.3\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\esp32ulp-elf\2.38_20240113\esp32ulp-elf\bin;C:\Espressif\tools\openocd-esp32\v0.12.0-esp32-20260703\openocd-esp32\bin;C:\Espressif\tools\dfu-util\0.11\dfu-util-0.11-win64;C:\Espressif\tools\idf-exe\1.0.3;%PATH%
python "%IDF_PATH%\tools\idf.py" %*
