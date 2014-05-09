@echo off

del *.wixobj

candle.exe Ryan_Debug.wxs 
REM -o installer/Ryan_Debug.wixobj

CALL collectdirs.bat

for %%r IN (installer/config*.wxs) DO candle.exe -arch x64 installer/%%r -dvar.MySource=installer
rem for %%r IN (installer/sxs*.wxs) DO candle.exe -arch x64 installer/%%r -dvar.MySource=installer
REM -o installer/%%r

light.exe -ext WixUIExtension -sw1076 *.wixobj -o Ryan_Debug.msi
signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll Ryan_Debug.msi

