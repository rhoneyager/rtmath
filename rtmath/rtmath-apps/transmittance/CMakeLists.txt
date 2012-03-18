# Create the test program and link it with the rtmath library

include(../../FindROOT.cmake)

add_executable (rtmath-transmittance transmittance.cpp)
target_link_libraries (rtmath-transmittance rtmath-static)

IF(${UNIX})
target_link_libraries (rtmath-transmittance netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-transmittance ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-transmittance PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})


