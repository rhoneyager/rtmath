# Create the test program and link it with the rtmath library

include(../../FindROOT.cmake)

add_executable (rtmath-absorb-slab absorb-slab.cpp)
target_link_libraries (rtmath-absorb-slab rtmath-static)

IF(${UNIX})
target_link_libraries (rtmath-absorb-slab netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-absorb-slab ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-absorb-slab PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})


