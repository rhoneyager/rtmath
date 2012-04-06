# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-getm getm.cpp)
set(appname rtmath-getm)
include (../../CMakeRules/addapp.cmake)

#INSTALL(TARGETS rtmath-getm RUNTIME DESTINATION bin COMPONENT Applications)
INSTALL(TARGETS rtmath-getm RUNTIME DESTINATION bin RENAME rtmath-genmtab COMPONENT Applications)

