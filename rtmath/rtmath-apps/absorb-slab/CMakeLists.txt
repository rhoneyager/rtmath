# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-absorb-slab absorb-slab.cpp)
set(appname rtmath-absorb-slab)
include (../../CMakeRules/addapp.cmake)