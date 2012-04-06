# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-da da.cpp)
set(appname rtmath-da)
include (../../CMakeRules/addapp.cmake)