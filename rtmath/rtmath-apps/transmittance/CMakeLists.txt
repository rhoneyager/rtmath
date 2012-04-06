# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-transmittance transmittance.cpp)
set(appname rtmath-transmittance)
include (../../CMakeRules/addapp.cmake)
