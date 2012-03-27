# Create the rtmath library

# Really should specify the files manually, 
# but I can just delete the build and recreate for now.
aux_source_directory (src libsrcs)

# Find ROOT directories and files
#include(../FindROOT.cmake)
include (../CMakeRules/common.cmake)

# Options
option ( USE_IPP
	"Use Intel Integrated Performance Primitives" OFF)
option ( USE_OPENMP
	"Enable OpenMP to speed up execution in certain areas" OFF)

# Custom prebuild step based on os

IF(${UNIX})
	add_custom_command (
		COMMAND tcsh ./prebuild.csh "${CMAKE_CURRENT_BINARY_DIR}"
		OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
ENDIF(${UNIX})
IF(${WIN32})
	add_custom_command (
		COMMAND prebuild.bat "${CMAKE_CURRENT_BINARY_DIR}"
		COMMENT Running custom commands to set paths and query the repository information.
		OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
ENDIF(${WIN32})

# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")


add_library (rtmath SHARED ${libsrcs} ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h)

add_library (rtmath-static STATIC ${libsrcs} ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h)

aux_source_directory (tests modtests)
add_executable (rtmath-tests ${modtests})
target_link_libraries (rtmath-tests rtmath-static)

IF(${UNIX})
target_link_libraries (rtmath-tests netcdf boost_filesystem boost_unit_test_framework boost_system)
ENDIF(${UNIX})


# Ensure that the dynamic library builds first (so prebuild will have run)
add_dependencies (rtmath-tests rtmath-static)

# MSVC parallel builds by default
IF(${MSVC})
set_target_properties(rtmath PROPERTIES COMPILE_FLAGS /MP)
set_target_properties(rtmath-static PROPERTIES COMPILE_FLAGS /MP)
ENDIF(${MSVC})

# Use C++11 in gcc
IF(${CMAKE_COMPILER_IS_GNUCXX})
set_target_properties(rtmath PROPERTIES COMPILE_FLAGS -std=c++0x)
set_target_properties(rtmath-static PROPERTIES COMPILE_FLAGS -std=c++0x)
set_target_properties(rtmath-tests PROPERTIES COMPILE_FLAGS -std=c++0x)
ENDIF(${CMAKE_COMPILER_IS_GNUCXX})

# Add in root dependencies on unix (AND WINDOWS)
IF(${UNIX})
include_directories (AFTER SYSTEM ${ROOT_INCLUDES})
ENDIF(${UNIX})


# Install targets. This ensures that the library is in a valid location, 
# and places the testing program in the appropriate directory.

# Headers
INSTALL(DIRECTORY rtmath DESTINATION include)

# Executables and libraries
INSTALL(TARGETS rtmath-tests rtmath-static rtmath 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)


