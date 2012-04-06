# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-ddparse ddparse2.cpp)
set(appname rtmath-ddparse)
include (../../CMakeRules/addapp.cmake)