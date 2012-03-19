# Common include file for all rtmath-derived projects that use CMake.

# Loads the necessary components for libraries and sets various 
# compiler flags. 

# Find the locations of ROOT, OpenMP and others
include (${CMAKE_CURRENT_LIST_DIR}/FindROOT.cmake)
#include (${CMAKE_CURRENT_LIST_DIR}/FindOpenMP.cmake)

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})

# Set libraries on unix (MSVC provides automatically)
IF(${UNIX})
SET (COMMON_LIBS netcdf m boost_filesystem boost_unit_test_framework boost_system ${ROOT_LIBRARIES})
ENDIF()

# Enable C++11
IF(${CMAKE_COMPILER_IS_GNUCXX})
SET (COMMON_CFLAGS ${COMMON_CFLAGS} -std=c++0x)
ENDIF()

# Take OpenMP option and enable / disable as appropriate
IF(${USE_OPENMP})
IF (${OPENMP_FOUND})
SET (COMMON_CFLAGS ${COMMON_CFLAGS} ${OpenMP_CXX_FLAGS})
ENDIF()
ENDIF()

