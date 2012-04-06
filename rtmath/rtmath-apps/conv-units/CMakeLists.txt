# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-conv-units conv.cpp)
set(appname rtmath-conv-units)
include (../../CMakeRules/addapp.cmake)