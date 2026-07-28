$env:PATH = "$env:USERPROFILE\.platformio\packages\toolchain-riscv32-esp@src-aaa7c2c8c1eddd6a5652007f592a4b1b\riscv32-esp-elf\bin;" + $env:PATH
pio run --target upload --target monitor --upload-port COM6 --monitor-port COM6
#\\.\build.ps1 in powershell to run