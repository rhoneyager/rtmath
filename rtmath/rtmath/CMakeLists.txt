# Create the rtmath library

# Really should specify the files manually, 
# but I can just delete the build and recreate for now.
aux_source_directory (src libsrcs)
#aux_source_directory (rtmath headers)

include (common)

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

# Win32 MSVC builds really prefer a static lib build.
IF(${WIN32})
	add_library (rtmath STATIC ${libsrcs} ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h)
ELSE()
	add_library (rtmath SHARED ${libsrcs} ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h)
ENDIF()

add_library (rtmath-static STATIC ${libsrcs} ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h)

aux_source_directory (tests modtests)
add_executable (rtmath-tests ${modtests})
target_link_libraries (rtmath-tests rtmath-static)

IF(${MSVC})
# MSVC parallel builds by default
SET(COMMON_CFLAGS ${COMMON_CFLAGS} /MP)
ENDIF(${MSVC})

#message(${COMMON_CFLAGS})

target_link_libraries (rtmath-tests ${COMMON_LIBS})
IF(EXISTS(COMMON_CFLAGS))
set_target_properties(rtmath PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
set_target_properties(rtmath-static PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
set_target_properties(rtmath-tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
ENDIF()

set_target_properties(rtmath PROPERTIES FOLDER "Libs")
set_target_properties(rtmath-static PROPERTIES FOLDER "Libs")
set_target_properties(rtmath-tests PROPERTIES FOLDER "Tests")

#add_header_files(headers)

# Ensure that the dynamic library builds first (so prebuild will have run)
add_dependencies (rtmath-tests rtmath-static)


# Install targets. This ensures that the library is in a valid location, 
# and places the testing program in the appropriate directory.

# Headers
INSTALL(DIRECTORY rtmath 
	DESTINATION include
	COMPONENT Headers
	PATTERN ".svn" EXCLUDE
	)

# Executables and libraries
INSTALL(TARGETS rtmath-tests
	RUNTIME DESTINATION bin
	COMPONENT Tests
	)

INSTALL(TARGETS rtmath-static rtmath 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	COMPONENT Libraries
	)


