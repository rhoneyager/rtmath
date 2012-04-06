# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-sector sector.cpp)
set(appname rtmath-sector)
include (../../CMakeRules/addapp.cmake)