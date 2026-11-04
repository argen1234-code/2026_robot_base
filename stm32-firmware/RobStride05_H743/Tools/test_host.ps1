$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
Push-Location $root
try {
 & gcc -std=c99 -Wall -Wextra -Werror Tools/test_dual_motor.c Devices/robstride05.c Devices/jy901s.c App/app_chassis.c App/app_pid.c App/app_bluetooth.c -ITools/mocks -IApp -IDevices -IBSP -lm -o Tools/test_dual_motor.exe
 if($LASTEXITCODE -ne 0){throw 'Dual motor host compilation failed'}
 & ./Tools/test_dual_motor.exe
 if($LASTEXITCODE -ne 0){throw 'Dual motor host checks failed'}
 & gcc -std=c99 -Wall -Wextra -Werror -Wno-format-truncation Tools/test_display.c App/app_display.c -IApp -IBSP -IDevices -o Tools/test_display.exe
 if($LASTEXITCODE -ne 0){throw 'Display compilation failed'}
 & ./Tools/test_display.exe
 if($LASTEXITCODE -ne 0){throw 'Display checks failed'}
 & gcc -std=c99 -Wall -Wextra -Werror Tools/test_led.c -ITools/mocks -IApp -IBSP -IDevices -o Tools/test_led.exe
 if($LASTEXITCODE -ne 0){throw 'LED compilation failed'}
 & ./Tools/test_led.exe
 if($LASTEXITCODE -ne 0){throw 'LED checks failed'}
 Write-Output 'ALL HOST CHECKS PASSED'
} finally {Pop-Location}
