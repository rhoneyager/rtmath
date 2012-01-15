@echo off
subwcrev . "rtmath/error/debug_subversion.h.template" "build/debug_subversion.h" -f


rem ###echo #define RTC "%USERPROFILE%\.rtmath" > build/rtc.H
set ln=%USERPROFILE%\.rtmath
set lnb=%ln:\=/%
echo #define RTC "%lnb%" > build/rtc.h

