# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-version rtmath-version.cpp)
set(appname rtmath-version)
include (../../CMakeRules/addapp.cmake)
