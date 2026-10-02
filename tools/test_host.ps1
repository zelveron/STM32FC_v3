param([string]$VcVars = 'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot
$build=Join-Path $root 'build\host'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$shared=@(Get-ChildItem "$root\src\control\*.cpp","$root\src\modes\*.cpp","$root\src\estimation\*.cpp" | ForEach-Object FullName)
$core=@('arming','failsafe','scheduler','log_ring','log_frame') | ForEach-Object { "$root\src\core\$_.cpp" }
$reg=$shared+$core+@("$root\tests\regression.cpp","$root\src\drivers\crsf.cpp","$root\src\drivers\ublox.cpp")
$targets=@{
  native=($shared+$core+@("$root\src\main_native.cpp","$root\src\hal\native\hal_native.cpp"))
  sitl=($shared+@("$root\src\main_sitl.cpp","$root\src\sitl\aircraft.cpp","$root\src\sitl\sensors.cpp"))
  improvements=($shared+@("$root\tests\control_improvements.cpp"))
  scenarios=($shared+@("$root\tests\flight_scenarios.cpp","$root\src\sitl\aircraft.cpp","$root\src\sitl\sensors.cpp"))
  regression=$reg
  bench=$reg
  sd_timeout=@("$root\tests\sd_timeout.cpp")
  sd_logging=@("$root\tests\async_storage.cpp","$root\src\core\sd_bin_log.cpp","$root\src\core\log_ring.cpp","$root\src\core\log_frame.cpp")
  bmi_driver=@("$root\tests\bmi270_driver.cpp","$root\src\drivers\bmi270.cpp","$root\src\drivers\imu_v2.cpp","$root\src\estimation\imu_prep.cpp","$root\src\estimation\ahrs.cpp")
  bmi323_driver=@("$root\tests\bmi323_driver.cpp","$root\src\drivers\bmi323_fifo.cpp","$root\src\drivers\imu_v2.cpp","$root\src\estimation\imu_prep.cpp","$root\src\estimation\ahrs.cpp")
  bmp_driver=@("$root\tests\bmp581_driver.cpp","$root\src\drivers\bmp581.cpp")
}
$cmds=@('@echo off',('call "'+$VcVars+'" >nul'),'if errorlevel 1 exit /b 1')
$cmds+=('cl /nologo /O2 /c /I"'+$root+'\lib\bmi270\src" "'+$root+'\lib\bmi270\src\bmi2.c" /Fobmi2_vendor.obj > bosch-build.txt 2>&1'),'if errorlevel 1 exit /b 1'
$cmds+=('cl /nologo /O2 /c /I"'+$root+'\lib\bmi270\src" "'+$root+'\lib\bmi270\src\bmi270.c" /Fobmi270_vendor.obj >> bosch-build.txt 2>&1'),'if errorlevel 1 exit /b 1'
$cmds+=('cl /nologo /O2 /c /I"'+$root+'\lib\bmp5\src" "'+$root+'\lib\bmp5\src\bmp5.c" /Fobmp5_vendor.obj > bmp5-build.txt 2>&1'),'if errorlevel 1 exit /b 1'
$cmds+=('cl /nologo /O2 /c /I"'+$root+'\lib\bmi323\src" "'+$root+'\lib\bmi323\src\bmi3.c" /Fobmi3_vendor.obj > bmi323-build.txt 2>&1'),'if errorlevel 1 exit /b 1'
$cmds+=('cl /nologo /O2 /c /I"'+$root+'\lib\bmi323\src" "'+$root+'\lib\bmi323\src\bmi323.c" /Fobmi323_vendor.obj >> bmi323-build.txt 2>&1'),'if errorlevel 1 exit /b 1'
foreach($name in @('native','sitl' ,'regression','bench','sd_timeout','sd_logging','bmi_driver','bmi323_driver','bmp_driver','improvements','scenarios')) {
    $flags=@('/nologo','/std:c++17','/EHsc','/W3','/O2','/D_USE_MATH_DEFINES','/D_CRT_SECURE_NO_WARNINGS',('/I"'+$root+'\tests\stubs"'),('/Fe:'+$name+'.exe'))
    if($name -eq 'regression'){$flags+='/DFC_FLIGHT_ENABLED=1','/DFC_SD_LOGGING=1'}
    if($name -eq 'bmi_driver'){$flags+=('/I"'+$root+'\lib\bmi270\src"'),'bmi2_vendor.obj','bmi270_vendor.obj'}
    if($name -eq 'bmi323_driver'){$flags+=('/I"'+$root+'\lib\bmi323\src"'),'/DFC_IMU_BMI323=1','/DFC_MAG_ENABLED=0','bmi3_vendor.obj','bmi323_vendor.obj'}
    if($name -eq 'bmp_driver'){$flags+=('/I"'+$root+'\lib\bmp5\src"'),'bmp5_vendor.obj'}
    $flags+=($targets[$name] | ForEach-Object {'"'+$_+'"'})
    Set-Content -LiteralPath "$build\$name.rsp" -Value $flags -Encoding ascii
    $cmds+=("cl @"+$name+".rsp > "+$name+"-build.txt 2>&1"),'if errorlevel 1 exit /b 1'
}
Set-Content -LiteralPath "$build\compile.cmd" -Value $cmds -Encoding ascii
Push-Location $build
try {
    & $env:ComSpec /d /c compile.cmd
    if($LASTEXITCODE -ne 0){Get-Content *-build.txt;throw 'Host compilation failed'}
    foreach($name in @('native','regression','bench','sd_timeout','sd_logging','bmi_driver','bmi323_driver','bmp_driver','improvements','scenarios')) {
        & ".\$name.exe" | Tee-Object "$name-results.txt"
        if($LASTEXITCODE -ne 0){throw "$name tests failed"}
    }
    & $env:ComSpec /d /c 'sitl.exe --check > sitl-results.txt 2> sitl-metrics.txt'
    if($LASTEXITCODE -ne 0){throw 'SITL nominal failed'}
    & $env:ComSpec /d /c 'sitl.exe --assist-check > assist-results.txt 2> assist-metrics.txt'
    if($LASTEXITCODE -ne 0){throw 'SITL assist failed'}
    Get-Content sitl-results.txt,assist-results.txt
} finally {Pop-Location}
