param([string]$Python='python')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    $roots=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
    # The packaged profile is only the fallback used when no AXF is found next
    # to the EXE. Prefer the most recently built project so both the FreeRTOS
    # and the HAL firmware can be packaged; connect still verifies Flash.
    $firmware=@(
        (Join-Path $roots 'RobStride05_H743/MDK-ARM/Objects/RobStride05_H743.axf'),
        (Join-Path $roots 'RobStride05_H743_HAL/MDK-ARM/Objects/RobStride05_H743_HAL.axf')
    ) | Where-Object { Test-Path -LiteralPath $_ } |
        ForEach-Object { Get-Item -LiteralPath $_ } |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if (!$firmware) {throw 'Matching firmware AXF not found'}
    Write-Host "profile <- $($firmware.FullName)"
    & $Python -c "from profile import build_profile; from pathlib import Path; import json,sys; p=build_profile(sys.argv[1]); Path('firmware_profile.json').write_text(json.dumps(p,ensure_ascii=False),encoding='utf-8')" $firmware.FullName
    if($LASTEXITCODE -ne 0){throw 'Firmware profile failed'}
    $webData=(Join-Path $PSScriptRoot 'web')+';web'
    $profileData=(Join-Path $PSScriptRoot 'firmware_profile.json')+';.'
    & $Python -m PyInstaller --noconfirm --onefile --name RobStrideConsole --distpath '..' --workpath 'build/singlefile' --specpath 'build' --paths $PSScriptRoot --add-data $webData --add-data $profileData --collect-submodules pyocd --collect-data pyocd --copy-metadata pyocd --collect-all cmsis_pack_manager --collect-all libusb_package --collect-all capstone --hidden-import usb.backend.libusb1 --hidden-import hid --exclude-module tkinter --exclude-module matplotlib --exclude-module numpy --exclude-module IPython server.py
    if($LASTEXITCODE -ne 0){throw 'Single-file packaging failed'}
    Get-FileHash -LiteralPath '../RobStrideConsole.exe' -Algorithm SHA256
} finally {Pop-Location}
