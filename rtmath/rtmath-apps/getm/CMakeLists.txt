# Create the test program and link it with the rtmath library

include(../../FindROOT.cmake)

add_executable (rtmath-getm getm.cpp)
target_link_libraries (rtmath-getm rtmath)

IF(${UNIX})
target_link_libraries (rtmath-getm netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-getm ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-getm PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})

#INSTALL(TARGETS rtmath-getm RUNTIME DESTINATION bin)
INSTALL(TARGETS rtmath-getm RUNTIME DESTINATION bin RENAME rtmath-genmtab)

