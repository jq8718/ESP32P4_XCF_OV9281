param([string]$Python = "$PSScriptRoot/.venv/Scripts/python.exe")
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path "$PSScriptRoot/../..").Path
if (-not (Test-Path $Python)) {
    throw 'Create .venv and install requirements.txt, or pass an installed Python path with -Python.'
}
$buildNumberPath = Join-Path $PSScriptRoot 'build_number.txt'
$buildNumber = 0
if (Test-Path $buildNumberPath) {
    $buildNumber = [int](Get-Content -LiteralPath $buildNumberPath -Raw).Trim()
}
$buildNumber++
$buildVersion = '1.0.{0:D4}' -f $buildNumber
Set-Content -LiteralPath $buildNumberPath -Value $buildNumber -Encoding ASCII
$versionSource = "BUILD_VERSION = '$buildVersion'`n"
Set-Content -LiteralPath (Join-Path $PSScriptRoot 'build_version.py') -Value $versionSource -Encoding ASCII
& $Python -m PyInstaller --noconfirm --clean --onefile --windowed --name P4ScanOV9281Debugger `
    --distpath "$projectRoot/dist" --workpath "$PSScriptRoot/build" --specpath "$PSScriptRoot" `
    --exclude-module PySide6.QtQml --exclude-module PySide6.QtQuick --exclude-module PySide6.QtNetwork `
    --exclude-module PySide6.QtOpenGL --exclude-module PySide6.QtPrintSupport --exclude-module tkinter `
    "$PSScriptRoot/app.py"
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
Write-Host "Built P4ScanOV9281Debugger v$buildVersion"
