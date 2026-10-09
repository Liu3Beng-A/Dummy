param([switch]$OneFile)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
Push-Location -LiteralPath $projectRoot
try {
    python -m unittest discover -s tests -v
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed; build cancelled.' }
    python -m pip show PyInstaller | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Install packaging requirements first: python -m pip install -r packaging/requirements.txt' }
    if ($OneFile) {
        python -m PyInstaller --clean --onefile --windowed --noupx --name DummyRobotPortable --add-data 'resources:resources' --icon 'resources/icons/dummy.ico' --version-file 'packaging/version_info.txt' --specpath build/specs main.py
    } else {
        python -m PyInstaller --clean packaging/DummyRobot.spec
    }
    if ($LASTEXITCODE -ne 0) { throw 'PyInstaller failed.' }
} finally {
    Pop-Location
}
