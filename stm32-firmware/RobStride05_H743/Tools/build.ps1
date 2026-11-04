param([string]$Keil='C:\Keil_v5\UV4\UV4.exe',
      [string]$Python='python')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path $PSScriptRoot -Parent
& $Python "$PSScriptRoot\integrate.py"
if($LASTEXITCODE -ne 0){throw 'Integration failed'}
$mdk=Join-Path $projectRoot 'MDK-ARM'
$log=Join-Path $projectRoot 'Docs\build_output.log'
$started=Get-Date
$proc=Start-Process -FilePath $Keil -ArgumentList @('-r',('"'+$mdk+'\RobStride05_H743.uvprojx"'),'-o',('"'+$log+'"')) -WindowStyle Hidden -PassThru
if(-not $proc.WaitForExit(60000)){throw 'Keil still running; inspect build log before retrying'}
Get-Content -LiteralPath $log
$text=Get-Content -Raw -LiteralPath $log
$axf=Get-Item -LiteralPath "$mdk\Objects\RobStride05_H743.axf" -ErrorAction SilentlyContinue
if($text -notmatch '0 Error\(s\)' -or !$axf -or $axf.LastWriteTime -lt $started){throw 'Build failed or AXF was not rebuilt'}
Get-FileHash "$mdk\Objects\RobStride05_H743.axf","$mdk\Objects\RobStride05_H743.hex" -Algorithm SHA256
