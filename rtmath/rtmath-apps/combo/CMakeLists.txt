# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-combo combo2.cpp)
set(appname rtmath-combo)
include (../../CMakeRules/addapp.cmake)