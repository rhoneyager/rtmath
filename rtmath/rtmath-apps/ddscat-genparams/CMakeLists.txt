# Create the test program and link it with the rtmath library

include(../../FindROOT.cmake)

add_executable (rtmath-ddscat-genparams main.cpp Stdafx.cpp)
target_link_libraries (rtmath-ddscat-genparams rtmath-static)

IF(${UNIX})
target_link_libraries (rtmath-ddscat-genparams netcdf m boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-ddscat-genparams ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-ddscat-genparams PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})


