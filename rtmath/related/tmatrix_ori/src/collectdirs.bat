@echo off

rem Made instead by cmake install when deploying public assembly files
rem mkdir installer

rem Note that all of the heat entries have to be modified in the
rem candle call to set Win64!

echo Processing binary directories
for /d %%r IN (bin/*) DO CALL :procbindir %%r
echo Processing library directories
for /d %%r IN (lib/*) DO CALL :proclibdir %%r
echo Processing CMake directories
for /d %%r IN (CMake/*) DO CALL :proccmakedir %%r
echo Processing header files
for /d %%r IN (include/*) DO CALL :procincdir %%r
echo Processing source files
for /d %%r IN (src/*) DO CALL :procsrcdir %%r
echo Processing sxs files
for %%r IN (installer/sxs_*) DO CALL :procsxs %%r
echo All wxs files generated.

goto end

:procbindir
set lbl=%1
set lbl=%lbl:bin_=%
echo found %lbl%
heat.exe  dir bin/%1 -nologo -o installer/bin_%lbl%.wxs -ag -sw5150 -cg bin_%lbl% -dr bindir -var var.MySource
candle.exe -nologo -arch x64 installer/bin_%lbl%.wxs -dMySource=bin/%1

goto end

:procsxs
set lbl=%1
for /f "tokens=1,2,3,4 delims=. " %%a in ("%lbl%") do set junk1=%%a&set junk2=%%b&set partial=%%c&set junk4=%%d
for /f "tokens=1,2,3 delims=_ " %%a in ("%partial%") do set junk1=%%a&set arc=%%b&set cver=%%c
set tag=%arc%_%cver%
rem echo MySource bin/bin_%tag%
candle.exe -nologo -arch x64 installer/%1 -dMySource=bin/bin_%tag%

goto end

:proccmakedir
set lbl=%1
set lbl=%lbl:conf_=%
echo found %lbl%
heat.exe  dir CMake/%1 -nologo -o installer/conf_%lbl%.wxs -ag -sw5150 -cg conf_%lbl% -dr cmakedir -var var.MySource
candle.exe -nologo -arch x64 installer/conf_%lbl%.wxs -dMySource=CMake/%1

goto end

:proclibdir
set lbl=%1
set lbl=%lbl:lib_=%
echo found %lbl%
heat.exe  dir lib/%1 -nologo -o installer/lib_%lbl%.wxs -ag -sw5150 -cg lib_%lbl% -dr libdir -var var.MySource
candle.exe -nologo -arch x64 installer/lib_%lbl%.wxs -dMySource=lib/%1

goto end

:procincdir
set lbl=%1
rem set lbl=%lbl:lib_=%
echo found %lbl%
heat.exe  dir include/%1 -nologo -o installer/inc_%lbl%.wxs -ag -sw5150 -cg inc_%lbl% -dr includedir -var var.MySource
candle.exe -nologo -arch x64 installer/inc_%lbl%.wxs -dMySource=include/%1

goto end

:procsrcdir
set lbl=%1
rem set lbl=%lbl:lib_=%
echo found %lbl%
heat.exe  dir src/%1 -nologo -o installer/src_%lbl%.wxs -ag -sw5150 -cg src_%lbl% -dr srcdir -var var.MySource
candle.exe -nologo -arch x64 installer/src_%lbl%.wxs -dMySource=src/%1

goto end


:end

