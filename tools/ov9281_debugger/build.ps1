param([string]$Python = "$PSScriptRoot/.venv/Scripts/python.exe")
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path "$PSScriptRoot/../..").Path
if (-not (Test-Path $Python)) {
    throw '请先创建 .venv 并安装 requirements.txt，或用 -Python 指定已安装依赖的 Python。'
}
& $Python -m PyInstaller --noconfirm --clean --onefile --windowed --name P4ScanOV9281Debugger `
    --distpath "$projectRoot/dist" --workpath "$PSScriptRoot/build" --specpath "$PSScriptRoot" `
    --exclude-module PySide6.QtQml --exclude-module PySide6.QtQuick --exclude-module PySide6.QtNetwork `
    --exclude-module PySide6.QtOpenGL --exclude-module PySide6.QtPrintSupport --exclude-module tkinter `
    "$PSScriptRoot/app.py"
exit $LASTEXITCODE
