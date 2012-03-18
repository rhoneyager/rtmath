# Create the test program and link it with the rtmath library

include(../../FindROOT.cmake)

add_executable (rtmath-sector sector.cpp)
target_link_libraries (rtmath-sector rtmath-static)

IF(${UNIX})
target_link_libraries (rtmath-sector netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-sector ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-sector PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})


