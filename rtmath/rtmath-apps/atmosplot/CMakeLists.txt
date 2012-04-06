# Create the test program and link it with the rtmath library
include (../../CMakeRules/common.cmake)


add_executable (rtmath-atmosplot atmosplot.cpp)
set(appname rtmath-atmosplot)
include (../../CMakeRules/addapp.cmake)