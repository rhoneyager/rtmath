# Create the program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-mie main.cpp)
set(appname rtmath-mie)
include (../../CMakeRules/addapp.cmake)
