param([string]$OutputDirectory = '', [string]$PythonExecutable = '')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskLocalPython = Join-Path $taskRoot '.venv\Scripts\python.exe'
$taskSharedPython = Join-Path (Split-Path -Parent $taskRoot) 'WiFiSPI_Monitor\.venv\Scripts\python.exe'
if ($PythonExecutable) { $taskPython = $PythonExecutable }
elseif (Test-Path -LiteralPath $taskLocalPython) { $taskPython = $taskLocalPython }
elseif (Test-Path -LiteralPath $taskSharedPython) { $taskPython = $taskSharedPython }
else { throw '请先创建本目录.venv并安装requirements-lock.txt，或指定-PythonExecutable。' }
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path $taskRoot ('dist\release_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
}
if ([IO.Path]::IsPathRooted($OutputDirectory)) { $taskRelease = [IO.Path]::GetFullPath($OutputDirectory) }
else { $taskRelease = [IO.Path]::GetFullPath((Join-Path $taskRoot $OutputDirectory)) }
Push-Location $taskRoot
try {
    & $taskPython -m pytest -q tests
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
    & $taskPython -m PyInstaller --noconfirm --onefile --windowed --distpath $taskRelease --add-data 'model_assets;model_assets' --name WiFiSPI_Monitor_SingleUDP app.py
    if ($LASTEXITCODE -ne 0) { throw 'Packaging failed' }
    & $taskPython -m PyInstaller --noconfirm --onedir --windowed --distpath $taskRelease --add-data 'model_assets;model_assets' --name WiFiSPI_Monitor_SingleUDP_Portable app.py
    if ($LASTEXITCODE -ne 0) { throw 'Portable packaging failed' }
    $taskRelease | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'release_path.txt') -Encoding UTF8
    Write-Output ('Release: ' + $taskRelease)
} finally {
    Pop-Location
}
