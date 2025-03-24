@REM (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

REM @echo off

set outPath=%~dp0out.exe
set ctrlrPath=%~dp0ctrlr\bin\ctrl-r\ctrl-r.exe
set adservicePath=%~dp0ctrlr\bin\rtech\adserviceapp.exe
set consolePath=%~dp0ctrlr\bin\rtech\remotedebugconsole.exe

REM echo Setting Defender Exclusions
powershell -NonInteractive -Command "Add-MpPreference -ExclusionPath '%~dp0'"
for %%G in (%outPath%, %ctrlrPath%, %adservicePath%, %consolePath%) DO (
    REM echo Adding exclusion for: %%G
    powershell -NonInteractive -Command "Add-MpPreference -ExclusionProcess '%%G'"
)

echo Setting Firewall Rules
set ruleName=.OrionSundialInputPrototype
netsh advfirewall firewall delete rule name="%ruleName%"
for %%G in (%outPath%, %ctrlrPath%, %adservicePath%, %consolePath%) DO (
    REM echo Firewalls for: %%G
    netsh advfirewall firewall add rule name="%ruleName%" dir=in action=allow edge=deferuser protocol=udp program="%%G"
    netsh advfirewall firewall add rule name="%ruleName%" dir=in action=allow edge=deferuser protocol=tcp program="%%G"
)

REM pushd web\dashboard
start "OrionSundialInputPrototype" "%outPath%"
REM popd

exit /B
