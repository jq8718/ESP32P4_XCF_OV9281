param([string]$Python = 'py')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path "$PSScriptRoot/../..").Path
$buildNumberPath = Join-Path $PSScriptRoot 'build_number.txt'
$buildNumber = 0
if (Test-Path $buildNumberPath) { $buildNumber = [int](Get-Content -LiteralPath $buildNumberPath -Raw).Trim() }
$buildNumber++
$buildVersion = '1.0.{0:D4}' -f $buildNumber
Set-Content -LiteralPath $buildNumberPath -Value $buildNumber -Encoding ASCII
Set-Content -LiteralPath (Join-Path $PSScriptRoot 'build_version.py') -Value "BUILD_VERSION = '$buildVersion'`n" -Encoding ASCII
& $Python -3.12 -m PyInstaller --noconfirm --clean --onefile --windowed `
    --name "AGV_3D_Test" --distpath "$PSScriptRoot/release" `
    --workpath "$PSScriptRoot/build" --specpath "$PSScriptRoot" "$PSScriptRoot/app.py"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Host "Built AGV_3D_Test.exe (v$buildVersion)"
