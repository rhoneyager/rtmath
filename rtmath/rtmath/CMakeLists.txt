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


# Shared/static flag
if(WIN32 AND NOT CYGWIN)
	set (sflag STATIC)
else()
	set (sflag SHARED)
endif()

# Custom prebuild step based on os
IF(WIN32 AND NOT CYGWIN)
	add_custom_command (
		COMMAND prebuild.bat "${CMAKE_CURRENT_BINARY_DIR}"
		COMMENT Running custom commands to set paths and query the repository information.
		OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
else()
	add_custom_command (
		COMMAND tcsh ./prebuild.csh "${CMAKE_CURRENT_BINARY_DIR}"
		OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	)
ENDIF()

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()


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

find_package(Boost COMPONENTS program_options serialization iostreams filesystem system unit_test_framework regex REQUIRED)
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
#message("${Boost_FILESYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
#                ${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY}
#                ${Boost_PROGRAM_OPTIONS_LIBRARY}")
if(WIN32 AND NOT CYGWIN)
	option ( AUTOLINK_BOOST
		"Automatically link Boost" ON)
	if (AUTOLINK_BOOST)
		link_directories(${Boost_LIBRARY_DIR})
		set(rtmath_core-libs "")
	else()
		add_definitions(-DBOOST_ALL_NO_LIB)
		set(rtmath_core-libs ${Boost_FILESYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
		${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY})
	endif()
else()
	set (rtmath_core-libs ${Boost_FILESYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
		${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

find_package(Ryan_Debug REQUIRED)
find_package(Ryan_Serialization REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})

add_library (rtmath_core ${sflag}
	${rtmath_core-files} 
	${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h 
	${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h
	${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
	)
IF(DEFINED COMMON_CFLAGS )
	set_target_properties(rtmath_core PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
endif()
SET_TARGET_PROPERTIES( rtmath_core PROPERTIES RELEASE_POSTFIX _Release )
SET_TARGET_PROPERTIES( rtmath_core PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel )
SET_TARGET_PROPERTIES( rtmath_core PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo )
SET_TARGET_PROPERTIES( rtmath_core PROPERTIES DEBUG_POSTFIX _Debug )
set_target_properties(rtmath_core PROPERTIES FOLDER "Libs")




set (rtmath_core-libs ${rtmath_core-libs} Ryan_Debug Ryan_Serialization)
target_link_libraries(rtmath_core ${rtmath_core-libs})






add_library (rtmath_mie ${sflag}
	${rtmath_mie-files})
IF(DEFINED COMMON_CFLAGS )
	set_target_properties(rtmath_mie PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
endif()
SET_TARGET_PROPERTIES( rtmath_mie PROPERTIES RELEASE_POSTFIX _Release )
SET_TARGET_PROPERTIES( rtmath_mie PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel )
SET_TARGET_PROPERTIES( rtmath_mie PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo )
SET_TARGET_PROPERTIES( rtmath_mie PROPERTIES DEBUG_POSTFIX _Debug )
set_target_properties( rtmath_mie PROPERTIES FOLDER "Libs")
set (rtmath_mie-libs rtmath_core ${rtmath_core-libs})
target_link_libraries(rtmath_mie ${rtmath_mie-libs})




find_package(tmatrixFortran REQUIRED)
find_package(tmatrixCpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io segmentation surface)
include_directories(${TMATRIX_CPP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_library (rtmath_ddscat ${sflag}
	${rtmath_ddscat-files}
	)
IF(DEFINED COMMON_CFLAGS )
	set_target_properties(rtmath_ddscat PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
endif()
SET_TARGET_PROPERTIES( rtmath_ddscat PROPERTIES RELEASE_POSTFIX _Release )
SET_TARGET_PROPERTIES( rtmath_ddscat PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel )
SET_TARGET_PROPERTIES( rtmath_ddscat PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo )
SET_TARGET_PROPERTIES( rtmath_ddscat PROPERTIES DEBUG_POSTFIX _Debug )
set_target_properties( rtmath_ddscat PROPERTIES FOLDER "Libs")


#add_definitions(${PCL_DEFINITIONS})


set (rtmath_ddscat-libs rtmath_core ${rtmath_core-libs} rtmath_mie ${rtmath_mie-libs} 
	tmatrix-cpp tmatrix-fortran
	${Boost_REGEX_LIBRARY}	
	${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
	${PCL_SEGMENTATION_LIBRARIES} ${PCL_SURFACE_LIBRARIES})
target_link_libraries(rtmath_ddscat ${rtmath_ddscat-libs})




#add_dependencies(rtmath_ddscat ${rtmath_ddscat-libs})

#	add_package_dependency(rtmath DEPENDS_ON ROOT)
#	add_package_dependency(rtmath DEPENDS_ON VTK)
#	add_package_dependency(rtmath DEPENDS_ON PCL)
#	add_package_dependency(rtmath DEPENDS_ON ImageMagick)

#include_directories(${rtmath_INCLUDE_DIRS})
#set(rtmath_INCLUDES ${rtmath_INCLUDE_DIRS} CACHE INTERNAL "rtmath-included dirs")
#set(rtmath_LIBS ${rtmath_LIBRARIES} CACHE INTERNAL "rtmath-included libs")

#target_link_libraries (rtmath tmatrix-serialization rtmath-mie)

if (BUILD_TESTING)
	add_executable(rtmath_core_tests ${rtmath_core_test-files})
	add_executable(rtmath_mie_tests ${rtmath_mie_test-files})
	add_executable(rtmath_ddscat_tests ${rtmath_ddscat_test-files})
IF(DEFINED COMMON_CFLAGS )
	set_target_properties(rtmath_core_tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	set_target_properties(rtmath_mie_tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	set_target_properties(rtmath_ddscat_tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
endif()
set_target_properties( rtmath_core_tests PROPERTIES FOLDER "Tests")
target_link_libraries(rtmath_core_tests ${rtmath_core-libs} rtmath_core)
set_target_properties( rtmath_mie_tests PROPERTIES FOLDER "Tests")
target_link_libraries(rtmath_mie_tests ${rtmath_mie-libs} rtmath_mie)
set_target_properties( rtmath_ddscat_tests PROPERTIES FOLDER "Tests")
target_link_libraries(rtmath_ddscat_tests ${rtmath_ddscat-libs} rtmath_ddscat)

endif()
#target_link_libraries(rtmath-tests 
#	boost_filesystem 
#	boost_date_time 
#	boost_unit_test_framework 
#	boost_system
#)


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
	DESTINATION ${INSTALL_INCLUDE_DIR}
	COMPONENT Headers
	FILES_MATCHING PATTERN "*.h"
	)

IF(RTMATH-TESTS)
# Executables and libraries
INSTALL(TARGETS rtmath-tests
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Tests
	)
ENDIF()

INSTALL(TARGETS rtmath_core rtmath_ddscat rtmath_mie # rtmath-static
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	LIBRARY DESTINATION ${INSTALL_LIB_DIR}
	ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
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

