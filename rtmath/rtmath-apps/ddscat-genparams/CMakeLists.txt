# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-ddscat-genparams main.cpp Stdafx.cpp help.cpp)
set(appname rtmath-ddscat-genparams)
include (../../CMakeRules/addapp.cmake)
