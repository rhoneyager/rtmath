# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

# For now, don't build on windows
IF(${UNIX})
add_executable (rtmath-db-catalog-ddscat process_ddscat.cpp)
target_link_libraries (rtmath-db-catalog-ddscat rtmath)


target_link_libraries (rtmath-db-catalog-ddscat netcdf m boost_filesystem boost_unit_test_framework boost_system)


include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
target_link_libraries (rtmath-db-catalog-ddscat ${ROOT_LIBRARIES})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath-db-catalog-ddscat PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})

ENDIF(${UNIX})


#INSTALL(TARGETS rtmath-db-catalog-ddscat RUNTIME DESTINATION bin COMPONENT Applications)


