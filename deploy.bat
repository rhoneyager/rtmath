@echo off

candle.exe Ryan_Debug.wxs
light.exe -ext WixUIExtension -sw1076 Ryan_Debug.wixobj
signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll Ryan_Debug.msi

