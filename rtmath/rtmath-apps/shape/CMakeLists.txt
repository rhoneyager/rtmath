# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

add_executable (rtmath-shape shape-main.cpp)
target_link_libraries (rtmath-shape rtmath)

IF(${UNIX})
target_link_libraries (rtmath-shape netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-shape ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-shape PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})

INSTALL(TARGETS rtmath-shape RUNTIME DESTINATION bin COMPONENT Applications)

