@echo off

del *.wixobj

candle.exe @MSI_NAME@.wxs
CALL collectdirs.bat
for %%r IN (installer/config*.wxs) DO candle.exe -arch x64 installer/%%r -dvar.MySource=installer

light.exe -ext WixUIExtension -sw1076 *.wixobj -o @MSI_NAME@.msi
signtool.exe sign @TIMESTAMP_PROVIDER@ @MSI_NAME@.msi

