# Create the rtmath library

include (common)

# Really should specify the files manually, 
# but I can just delete the build and recreate for now.
#aux_source_directory (src libsrcs)
#aux_source_directory (rtmath headers)

# Setting this to prevent forced link of all symbols.
# I need this for a few builds where I am making extensive changes to the code.
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

include (files_src.cmake)

set (libsrcs ${srcfiles})


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

configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/rtmath/cmake-settings.h.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h"
	)

configure_file (
        "${CMAKE_CURRENT_SOURCE_DIR}/data/defaults/modulefiles/rtmath"
        "${CMAKE_CURRENT_BINARY_DIR}/module-rtmath"
        )

# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# Win32 MSVC builds really prefer a static lib build.
IF(${WIN32})
	add_library (rtmath STATIC 
			${libsrcs} 
			${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h 
			${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h
			${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
			)
ELSE()
	add_library (rtmath SHARED 
			${libsrcs} 
			${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h 
			${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h
			${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
			)
ENDIF()

# Used to allow VS default project to coexist with cmake project
add_definitions(-DWITH_CMAKE)

#add_library (rtmath-static STATIC 
#		${libsrcs} 
#		${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h 
#		${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h 
#		${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
#		)

get_target_property(rtmath_INCLUDE_DIRS rtmath INCLUDE_DIRECTORIES)
if (NOT rtmath_INCLUDE_DIRS)
	set(rtmath_INCLUDE_DIRS "")
	set(rtmath-tests_INCLUDE_DIRS "")
endif()
#get_target_property(rtmath_LIBRARIES rtmath LIBRARIES)
set(rtmath_LIBRARIES "")
set(rtmath-tests_LIBRARIES "")

#message(${rtmath_LIBRARIES})
#message(${rtmath_INCLUDE_DIRS})

if(LOAD_PACKAGES)	
	#add_package_dependency(rtmath DEPENDS_ON HDF5)
	add_package_dependency(rtmath DEPENDS_ON NetCDF)
#	add_package_dependency(rtmath DEPENDS_ON ROOT)
	add_package_dependency(rtmath DEPENDS_ON Boost)
	add_package_dependency(rtmath DEPENDS_ON VTK)
	add_package_dependency(rtmath DEPENDS_ON PCL)
	add_package_dependency(rtmath DEPENDS_ON ImageMagick)
	#add_package_dependency(rtmath DEPENDS_ON )
	#add_package_dependency(rtmath DEPENDS_ON )
endif()
IF(USE_OPENMP)
	IF (${OPENMP_FOUND})
		add_package_dependency(rtmath DEPENDS_ON OpenMP)
	endif()
endif()

include_directories(${rtmath_INCLUDE_DIRS})
set(rtmath_INCLUDES ${rtmath_INCLUDE_DIRS} CACHE INTERNAL "rtmath-included dirs")
set(rtmath_LIBS ${rtmath_LIBRARIES} CACHE INTERNAL "rtmath-included libs")

#message("${rtmath_INCLUDES}")
#message("${rtmath_LIBS}")

#add_package_dependency(rtmath-static DEPENDS_ON rtmathDummy)
#set(rtmath-static_LIBRARIES ${rtmath_LIBRARIES})
#set(rtmath-static_INCLUDE_DIRS ${rtmath_INCLUDE_DIRS})
target_link_libraries (rtmath ${COMMON_LIBS})
target_link_libraries (rtmath ${rtmath_LIBRARIES})
#target_link_libraries (rtmath-static ${COMMON_LIBS})
#target_link_libraries (rtmath-static ${rtmath-static_LIBRARIES})

aux_source_directory (tests modtests)
add_executable (rtmath-tests ${modtests})
# Ensure that the dynamic library builds first (so prebuild will have run)
add_dependencies (rtmath-tests rtmath)
target_link_libraries (rtmath-tests rtmath)
#target_link_libraries(rtmath-tests 

#add_package_dependency(rtmath-tests DEPENDS_ON rtmathDummy)
set(rtmath-tests_LIBRARIES ${rtmath_LIBRARIES})
set(rtmath-tests_INCLUDE_DIRS ${rtmath_INCLUDE_DIRS})
add_package_dependency(rtmath-tests DEPENDS_ON ROOT)

target_link_libraries (rtmath-tests ${COMMON_LIBS})
target_link_libraries (rtmath-tests ${rtmath-tests_LIBRARIES})

#target_link_libraries(rtmath-tests 
#	boost_filesystem 
#	boost_date_time 
#	boost_unit_test_framework 
#	boost_system
#)

IF(DEFINED COMMON_CFLAGS )
	set_target_properties(rtmath PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
#	set_target_properties(rtmath-static PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	set_target_properties(rtmath-tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
ENDIF()

set_target_properties(rtmath PROPERTIES FOLDER "Libs")
#set_target_properties(rtmath-static PROPERTIES FOLDER "Libs")
set_target_properties(rtmath-tests PROPERTIES FOLDER "Tests")

#add_header_files(headers)



# Install targets. This ensures that the library is in a valid location, 
# and places the testing program in the appropriate directory.

# Data directory
if (INSTALL_DATA)
INSTALL(DIRECTORY data
	DESTINATION ${DATA_DIR_PREFIX}
	COMPONENT Data
	PATTERN ".svn" EXCLUDE
	)
endif()

# Headers
INSTALL(DIRECTORY rtmath 
	DESTINATION include
	COMPONENT Headers
	PATTERN ".svn" EXCLUDE
	PATTERN ".in" EXCLUDE
	)

# Executables and libraries
INSTALL(TARGETS rtmath-tests
	RUNTIME DESTINATION bin
	COMPONENT Tests
	)

INSTALL(TARGETS rtmath # rtmath-static
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib${LIB_SUFFIX}
	ARCHIVE DESTINATION lib${LIB_SUFFIX}
	COMPONENT Libraries
	)

# Install modulefiles on linux / unix / mac
if (NOT WIN32)
	if (INSTALL_MODULES)
		INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
			DESTINATION ${ENV_MOD_DIR_PREFIX}
			COMPONENT Data
			RENAME ${MODULES_FILENAME}
			)
	endif()
endif()

