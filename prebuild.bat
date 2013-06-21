@echo off
echo "Doing prebuild"

set str=%1
for /f "useback tokens=*" %%a in ('%str%') do set str=%%~a
set "outfile=%str%/debug_subversion.h"
echo "Outputting to %outfile%"

subwcrev . "rtmath/error/debug_subversion.h.template" "%outfile%" -f

rem set ln=%USERPROFILE%\.rtmath
rem set lnb=%ln:\=/%
rem echo #define RTC "%lnb%" >> %1/debug_subversion.h

goto end

:error
echo Missing argument
echo usage prebuild.bat (output directory)

:end
echo.
echo Done.

