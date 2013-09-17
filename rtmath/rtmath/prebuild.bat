@echo off
echo "Doing prebuild"

set str=%1
for /f "useback tokens=*" %%a in ('%str%') do set str=%%~a
set "outfile=%str%/debug_subversion.h"
echo "Outputting to %outfile%"

subwcrev . "rtmath/error/debug_subversion.h.template" "%outfile%" -f

set ln=%USERPROFILE%\.rtmath
set lnb=%ln:\=/%
echo #define RTC "%lnb%" >> %1/debug_subversion.h

set lnc=%USERPROFILE%\rtmath.conf
set lnd=%lnc:\=/%
echo #define RTCB "%lnd%" >> %1/debug_subversion.h

set lne=%USERPROFILE%\rtmath\rtmath.conf
set lnf=%lne:\=/%
echo #define RTCC "%lnf%" >> %1/debug_subversion.h

goto end

:error
echo Missing argument
echo usage prebuild.bat (output directory)

:end
echo.
echo Done.

