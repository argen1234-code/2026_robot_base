$ErrorActionPreference = 'Stop'
$monitorUrl = 'http://127.0.0.1:8877/'
$monitorApi = $monitorUrl + 'api/frame-rates'
function Test-FrameRateServer {
    try {
        $snapshot = Invoke-RestMethod -Uri $monitorApi -TimeoutSec 2
        return $null -ne $snapshot.metrics
    } catch {
        return $false
    }
}
if (-not (Test-FrameRateServer)) {
    $pythonExe = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
    if (-not (Test-Path -LiteralPath $pythonExe)) {
        $pythonExe = (Get-Command python -ErrorAction Stop).Source
    }
    Start-Process -FilePath $pythonExe -ArgumentList 'frame_rates_server.py' -WorkingDirectory $PSScriptRoot -WindowStyle Hidden
    for ($attempt = 0; $attempt -lt 20; $attempt++) {
        Start-Sleep -Milliseconds 250
        if (Test-FrameRateServer) { break }
    }
}
if (-not (Test-FrameRateServer)) { throw '帧率监视服务启动失败，可能端口 8877 已被其他程序使用。' }
Start-Process $monitorUrl
