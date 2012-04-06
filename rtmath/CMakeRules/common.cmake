# Common include file for all rtmath-derived projects that use CMake.

# Loads the necessary components for libraries and sets various 
# compiler flags. 

# Add path for custom modules
#set(CMAKE_MODULE_PATH
#  ${CMAKE_MODULE_PATH}
#  "${PROJECT_SOURCE_DIR}/CMakeRules"
#)

# Find the locations of ROOT, OpenMP and others
#include (${CMAKE_CURRENT_LIST_DIR}/FindROOT.cmake)
#include (${CMAKE_CURRENT_LIST_DIR}/FindOpenMP.cmake)
include (FindROOT)
include (addapp)

include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
set (COMMON_LIBS ${COMMON_LIBS} ${ROOT_LIBRARIES})

# Add other libraries (MSVC provides automatically)
IF(${MSVC})
ELSE()
SET (COMMON_LIBS ${COMMON_LIBS} netcdf m boost_filesystem boost_unit_test_framework boost_system ${ROOT_LIBRARIES})
ENDIF()

# Enable C++11
# g++
IF(${CMAKE_COMPILER_IS_GNUCXX})
	if ("${COMMON_CFLAGS}" MATCHES "0x$")
	else()
	SET (COMMON_CFLAGS ${COMMON_CFLAGS} -std=c++0x)
	endif()
ENDIF()
# llvm
# TODO!!!

# Take OpenMP option and enable / disable as appropriate
IF(${USE_OPENMP})
IF (${OPENMP_FOUND})
SET (COMMON_CFLAGS ${COMMON_CFLAGS} ${OpenMP_CXX_FLAGS})
ENDIF()
ENDIF()

