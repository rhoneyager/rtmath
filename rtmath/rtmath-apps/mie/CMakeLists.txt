# Create the program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-mie main.cpp)
target_link_libraries (rtmath-mie rtmath)

IF(${UNIX})
target_link_libraries (rtmath-mie netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-mie ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-mie PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})

INSTALL(TARGETS rtmath-mie RUNTIME DESTINATION bin COMPONENT Applications)

