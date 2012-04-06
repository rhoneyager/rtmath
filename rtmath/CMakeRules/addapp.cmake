# CMake script for the very repetitive structure of 
# rtmath-apps CMakeLists.txt files.

# Take the variable {appname}, and link libraries,
# set properties and create an INSTALL target

target_link_libraries (${appname} rtmath ${COMMON_LIBS})
IF(EXISTS(COMMON_CFLAGS))
set_target_properties(${appname} PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
ENDIF()

INSTALL(TARGETS ${appname} RUNTIME DESTINATION bin COMPONENT Applications)
