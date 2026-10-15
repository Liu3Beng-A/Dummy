# Build all five firmware targets into the single repository output directory.
[CmdletBinding()]
param(
    [ValidatePattern('^[0-9]{10}$')]
    [string]$ReleaseId = '2026101403'
)
$ErrorActionPreference = 'Stop'
$repoPath = Split-Path -Parent $PSScriptRoot
$artifactPath = Join-Path $repoPath 'firmware_artifacts'
$batchId = [Guid]::NewGuid().ToString('N').Substring(0, 16).ToUpperInvariant()
$epoch = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds().ToString()
New-Item -ItemType Directory -Force -Path $artifactPath | Out-Null
foreach ($project in @('dummy-ref-core-fw', 'dummy-motor-fw')) {
    $sourcePath = Join-Path $repoPath "firmware_new/$project"
    $buildPath = Join-Path $artifactPath ".build/$project"
    $configureLog = Join-Path $artifactPath "$project-configure.log"
    $buildLog = Join-Path $artifactPath "$project-build.log"
    & cmake -S $sourcePath -B $buildPath -G Ninja -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "-DFW_RELEASE_ID=$ReleaseId" `
        "-DFW_BUILDSET=$batchId" "-DFW_BUILD_EPOCH=$epoch" 2>&1 | Tee-Object -FilePath $configureLog
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed: $project" }
    & cmake --build $buildPath --parallel 6 2>&1 | Tee-Object -FilePath $buildLog
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed: $project" }
}
Write-Host "Firmware release=$ReleaseId buildset=$batchId epoch=$epoch"
Write-Host "All firmware artifacts: $artifactPath"
