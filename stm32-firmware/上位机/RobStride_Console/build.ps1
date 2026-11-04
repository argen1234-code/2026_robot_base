param([string]$Python='python')
# Keep both entry points on the verified single-file build.
& "$PSScriptRoot/build-singlefile.ps1" -Python $Python
