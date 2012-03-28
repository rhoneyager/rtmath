# Create the test program and link it with the rtmath library
include (../../CMakeRules/common.cmake)


add_executable (rtmath-atmosplot atmosplot.cpp)
target_link_libraries (rtmath-atmosplot rtmath)

IF(${UNIX})
target_link_libraries (rtmath-atmosplot netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-atmosplot ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-atmosplot PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})

INSTALL(TARGETS rtmath-atmosplot RUNTIME DESTINATION bin COMPONENT Applications)

