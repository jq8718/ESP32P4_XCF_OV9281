$ErrorActionPreference = 'Stop'
$env:PROCESSOR_ARCHITECTURE = 'AMD64'
$env:PYTHONUTF8 = '1'
Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue
$env:IDF_PATH = 'C:\Users\jq163\esp\esp-idf'
$env:IDF_PYTHON_ENV_PATH = 'C:\Users\jq163\.espressif\python_env\idf5.4_py3.11_env'
$env:PATH = 'C:\Users\jq163\.espressif\tools\cmake\3.30.2\bin;C:\Users\jq163\.espressif\tools\ninja\1.12.1;C:\Users\jq163\.espressif\tools\ccache\4.10.2\ccache-4.10.2-windows-x86_64;C:\Users\jq163\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin;' + $env:PATH
& "$env:IDF_PYTHON_ENV_PATH\Scripts\python.exe" "$env:IDF_PATH\tools\idf.py" build
if ($LASTEXITCODE -ne 0) { throw "ESP-IDF build failed: $LASTEXITCODE" }
