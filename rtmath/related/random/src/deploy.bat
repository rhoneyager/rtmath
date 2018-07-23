@echo off

del *.wixobj
candle.exe Tmatrix_Random.wxs
CALL collectdirs.bat
for %%r IN (installer/config*.wxs) DO candle.exe -arch x64 installer/%%r -dvar.MySource=installer

light.exe -ext WixUIExtension -sw1076 *.wixobj -o Ryan_Tmatrix_Random.msi
signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll Ryan_Tmatrix_Random.msi

