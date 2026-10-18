param(
    [string]$ContainerName = "pai_ros2",
    [string]$RemoteDir = "/home/ubuntu/pai_ws/src/restaurant_delivery/debug_output",
    [string]$LocalDir = "$PSScriptRoot\debug_output"
)

Write-Host "Syncing debug outputs from $ContainerName ($RemoteDir) to $LocalDir..." -ForegroundColor Cyan

if (-not (Test-Path $LocalDir)) {
    New-Item -ItemType Directory -Force -Path $LocalDir | Out-Null
}

docker cp "${ContainerName}:${RemoteDir}/." "$LocalDir"

if ($LASTEXITCODE -eq 0) {
    Write-Host "Sync completed successfully!" -ForegroundColor Green
    Get-ChildItem -Path $LocalDir | Select-Object Name, Length, LastWriteTime
} else {
    Write-Host "Failed to sync files from container." -ForegroundColor Red
}
