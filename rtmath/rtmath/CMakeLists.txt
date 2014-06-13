# Create the rtmath library

include (common)
include(addconfiginfo)
include(addlib)
include(addAssembly)
include(generateAssemblyGuid)


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
# Library flag - add_definitions works with targets in and below this directory
#add_definitions("-DEXPORTING_RTMATH")
#set (libflag "/D EXPORTING_RTMATH")

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

# Needed here to properly configure
set(COMPRESS_ZLIB 0)
set(COMPRESS_GZIP 0)
set(COMPRESS_BZIP2 0)
set(COMPRESS_SZIP 0) # Only the HDF plugin uses SZIP

find_package(BZip2)
# BZIP2_FOUND requires that the debug libary be found, which is not desired.
if (NOT BZIP2_INCLUDE_DIR)
	message(STATUS "BZip2 libraries not found. Retrying with environment variables.")
	# FindBZip.cmake is annoying. It doesn't check the environment.
	if (DEFINED ENV{BZIP2_INCLUDE_DIR})
		set(BZIP2_INCLUDE_DIR $ENV{BZIP2_INCLUDE_DIR}
			CACHE PATH "BZip2 Include directory" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARY_DEBUG})
		#set(BZIP2_LIBRARY_DEBUG $ENV{BZIP2_LIBRARY_DEBUG}
		#	CACHE FILEPATH "BZip2 Debug library path" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARY_RELEASE})
		#set(BZIP2_LIBRARY_RELEASE $ENV{BZIP2_LIBRARY_RELEASE}
		#	CACHE FILEPATH "BZip2 Release library path" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARIES})
		set(BZIP2_LIBRARIES $ENV{BZIP2_LIBRARIES}
			CACHE FILEPATH "BZip2 libraries" FORCE)
		mark_as_advanced(BZIP2_LIBRARIES)
	endif()
	find_package(BZip2)
endif()
find_package(ZLIB)


if (ZLIB_FOUND)
	set(COMPRESS_ZLIB 1)
	set(COMPRESS_GZIP 1)
	include_directories(${ZLIB_INCLUDE_DIRS})
	#set(liblist ${liblist} ${ZLIB_LIBRARIES})
endif()
# BZIP2_FOUND requires that the debug libary be found, which is not desired.
if (BZIP2_INCLUDE_DIR)
	set(COMPRESS_BZIP2 1)
	include_directories(${BZIP2_INCLUDE_DIR})
	#set(liblist ${liblist} ${BZIP2_LIBRARIES})
endif()
IF (DEFINED SZIP_LIBRARY)
	include_directories(${SZIP_INCLUDE_DIRS})
	#	SET (h5libs ${h5libs} szip)
	set(COMPRESS_SZIP 1)
ENDIF()

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

addBoostUniform(program_options serialization iostreams filesystem system unit_test_framework regex)
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
if(WIN32 AND NOT CYGWIN)
	set(rtmath_core-libs "")
else()
	set (rtmath_core-libs ${Boost_FILESYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY}
		${Boost_IOSTREAMS_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_REGEX_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

find_package(Ryan_Debug REQUIRED)
find_package(Ryan_Serialization REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
include_directories(${RYAN_SERIALIZATION_INCLUDE_DIRS})

add_library (rtmath_core SHARED
	${rtmath_core-files} 
	${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h 
	${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h
	${CMAKE_CURRENT_BINARY_DIR}/module-rtmath
	)
addlib(rtmath_core SHARED)
add_precompiled_header(rtmath_core Stdafx-core.h src)
set (rtmath_core-libs ${rtmath_core-libs} Ryan_Debug Ryan_Serialization)
if(WIN32 AND NOT CYGWIN)
else()
	set (rtmath_core-libs ${rtmath_core-libs} dl)
endif()
target_link_libraries(rtmath_core ${rtmath_core-libs})
storebin(rtmath_core)

#add_library (rtmath_mie SHARED
#	${rtmath_mie-files})
#addlib(rtmath_mie SHARED)
#add_precompiled_header(rtmath_mie Stdafx-mie.h src)
#set (rtmath_mie-libs rtmath_core ${rtmath_core-libs})
#target_link_libraries(rtmath_mie ${rtmath_mie-libs})
#storebin(rtmath_core)



find_package(Voro REQUIRED)
include_directories(${VORO_INCLUDE_DIRS})
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})
#find_package(Qhull REQUIRED)
#include_directories(${QHULL_INCLUDE_DIRS})

add_library (rtmath_ddscat_base SHARED
	${rtmath_ddscat_base-files}
	)
addlib(rtmath_ddscat_base SHARED)
add_precompiled_header(rtmath_ddscat_base Stdafx-ddscat_base.h src)
set (rtmath_ddscat_base-libs 
	rtmath_core ${rtmath_core-libs} 
	#	rtmath_mie ${rtmath_mie-libs} 
	)
target_link_libraries(rtmath_ddscat_base ${rtmath_ddscat_base-libs})
storebin(rtmath_ddscat_base)


add_library(rtmath_voronoi SHARED ${rtmath_voronoi-files})
addlib(rtmath_voronoi SHARED)
add_precompiled_header(rtmath_voronoi Stdafx-voronoi.h src)
set (rtmath_voronoi-libs rtmath_core ${rtmath_core-libs} 
	rtmath_ddscat_base ${rtmath_ddscat_base-libs}
	${VTK_LIBRARIES}
	${VORO_LIBRARIES}
	)
	#	${QHULL_LIBRARIES}
target_link_libraries(rtmath_voronoi ${rtmath_voronoi-libs})
storebin(rtmath_voronoi)


add_library (rtmath_ddscat SHARED
	${rtmath_ddscat-files}
	)
addlib(rtmath_ddscat SHARED)
add_precompiled_header(rtmath_ddscat Stdafx-ddscat.h src)
set (rtmath_ddscat-libs 
	rtmath_core ${rtmath_core-libs} 
	#	rtmath_mie ${rtmath_mie-libs}
        rtmath_ddscat_base ${rtmath_ddscat_base-libs}	
	rtmath_voronoi ${rtmath_voronoi-libs}
	)
target_link_libraries(rtmath_ddscat ${rtmath_ddscat-libs})
storebin(rtmath_ddscat)

add_library(rtmath_data SHARED ${rtmath_data-files})
addlib(rtmath_data SHARED)
add_precompiled_header(rtmath_data Stdafx-data.h src)
set (rtmath_data-libs rtmath_core ${rtmath_core-libs}
	rtmath_voronoi ${rtmath_voronoi-libs}
	)
target_link_libraries(rtmath_data ${rtmath_data-libs})
storebin(rtmath_data)


add_library(rtmath_rt SHARED ${rtmath_rt-files})
addlib(rtmath_rt SHARED)
add_precompiled_header(rtmath_rt Stdafx-rt.h src)
set (rtmath_rt-libs rtmath_core ${rtmath_core-libs}
	)
target_link_libraries(rtmath_rt ${rtmath_rt-libs})
storebin(rtmath_rt)

add_library(rtmath_images SHARED ${rtmath_images-files})
addlib(rtmath_images SHARED)
add_precompiled_header(rtmath_images Stdafx-images.h src)
set (rtmath_images-libs rtmath_core ${rtmath_core-libs}
	rtmath_voronoi ${rtmath_voronoi-libs}
	)
target_link_libraries(rtmath_images ${rtmath_images-libs})
storebin(rtmath_images)


#add_library(rtmath_da ${sflag} ${rtmath_da-files})
#addlib(rtmath_da)
#add_precompiled_header(rtmath_da Stdafx-da.h src)
#set (rtmath_da-libs rtmath_core ${rtmath_core-libs} 
#	)
#target_link_libraries(rtmath_da ${rtmath_da-libs})


#add_library(rtmath_mc ${sflag} ${rtmath_mc-files})
#addlib(rtmath_mc)
#add_precompiled_header(rtmath_mc Stdafx-mc.h src)
#set (rtmath_mc-libs rtmath_core ${rtmath_core-libs} 
#	)
#target_link_libraries(rtmath_mc ${rtmath_mc-libs})


if (BUILD_TESTING)
	add_executable(rtmath_core_tests ${rtmath_core_test-files})
	storebin(rtmath_core_tests)

	#add_executable(rtmath_mie_tests ${rtmath_mie_test-files})
	add_executable(rtmath_ddscat_tests ${rtmath_ddscat_test-files})
	storebin(rtmath_ddscat_tests)

IF(DEFINED COMMON_CFLAGS )
	#set_target_properties(rtmath_core_tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	#set_target_properties(rtmath_mie_tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	#set_target_properties(rtmath_ddscat_tests PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
endif()
set_target_properties( rtmath_core_tests PROPERTIES FOLDER "Tests")
#target_link_libraries(rtmath_core_tests ${rtmath_core-libs} rtmath_core ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
target_link_libraries(rtmath_core_tests ${rtmath_ddscat-libs} rtmath_core rtmath_mie rtmath_ddscat ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#set_target_properties( rtmath_mie_tests PROPERTIES FOLDER "Tests")
#target_link_libraries(rtmath_mie_tests ${rtmath_ddscat-libs} rtmath_core rtmath_mie rtmath_ddscat ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
set_target_properties( rtmath_ddscat_tests PROPERTIES FOLDER "Tests")
target_link_libraries(rtmath_ddscat_tests ${rtmath_ddscat-libs} rtmath_core rtmath_mie rtmath_ddscat ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

endif()

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
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	COMPONENT Tests
	)
ENDIF()

INSTALL(TARGETS rtmath_core rtmath_ddscat rtmath_ddscat_base 
	rtmath_voronoi rtmath_data rtmath_images
	# rtmath_mie
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
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

