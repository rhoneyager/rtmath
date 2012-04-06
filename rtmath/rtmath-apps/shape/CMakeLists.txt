# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-shape shape-main.cpp)
set(appname rtmath-shape)
include (../../CMakeRules/addapp.cmake)