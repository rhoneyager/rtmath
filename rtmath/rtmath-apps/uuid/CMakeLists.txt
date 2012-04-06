# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-uuid uuid.cpp)
set(appname rtmath-uuid)
include (../../CMakeRules/addapp.cmake)
