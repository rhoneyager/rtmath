cmake_minimum_required (VERSION 3.1)
project(rtmath C CXX)

INCLUDE(CTest)
ENABLE_TESTING()

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_SOURCE_DIR}/CMakeRules"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(addBaseProject)
include(addBoostUniform)
include(adddocs)
include(addapp)
INCLUDE(addlib)
include(generaterc)
include(GetGitRevisionDescription)
include(adddocs)
get_git_head_revision(GITREFSPECVAR GITHASHVAR)
set(MAJOR 0)
set(MINOR 5)
set(REVISION 1)
set(RTMATH_VERSION ${MAJOR}.${MINOR}.${REVISION})
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
addBaseProject()

configure_file (
    "${CMAKE_CURRENT_SOURCE_DIR}/rtmath/rtmath/rtmath/cmake-settings.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h"
    )

# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(REGISTRY_PLUGINS_OVERRIDE_DIR ../plugins CACHE PATH "Additional plugin search directory (leave to ../plugins for none)")

addBoostUniform(filesystem iostreams program_options system log log_setup unit_test_framework regex date_time)
if(WIN32 AND NOT CYGWIN)
	set( DATA_DIR_PREFIX_CAND
		C:/rtmath )
else()
	set( DATA_DIR_PREFIX_CAND
		/opt/rtmath/data )
endif()

SET( DATA_DIR_PREFIX
	${DATA_DIR_PREFIX_CAND}
	CACHE STRING
	"Data directory prefix for basic configuration"
)

option ( INSTALL_DATA
	"Install test data in the data directory" OFF)
option ( BUILD_APPS
	"Build the default applications" OFF)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Needed here to properly configure
set(COMPRESS_ZLIB 0)
set(COMPRESS_GZIP 0)
set(COMPRESS_BZIP2 0)
set(COMPRESS_SZIP 0) # Only the HDF plugin uses SZIP

if(NOT ZLIB_INCLUDE_DIR)
	if (DEFINED ENV{ZLIB_ROOT})
		set(ZLIB_ROOT $ENV{ZLIB_ROOT})
	ENDIF()
endif()

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
		set(BZIP2_LIBRARY_DEBUG $ENV{BZIP2_LIBRARY_DEBUG}
			CACHE FILEPATH "BZip2 Debug library path" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARY_RELEASE})
		set(BZIP2_LIBRARY_RELEASE $ENV{BZIP2_LIBRARY_RELEASE}
			CACHE FILEPATH "BZip2 Release library path" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARIES})
		set(BZIP2_LIBRARIES $ENV{BZIP2_LIBRARIES}
			CACHE FILEPATH "BZip2 libraries" FORCE)
		mark_as_advanced(BZIP2_LIBRARIES)
	endif()
	find_package(BZip2)
endif()


if(NOT ZLIB_INCLUDE_DIR)
	if (DEFINED ENV{ZLIB_ROOT})
		set(ZLIB_ROOT $ENV{ZLIB_ROOT})
	ENDIF()
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




add_subdirectory (ryan-debug)
add_subdirectory (rtmath/rtmath)
#add_subdirectory (rtmath/plugins)
if (BUILD_APPS)
	add_subdirectory (rtmath/rtmath-apps)
endif()


adddocs()

set(BUILDNAME "Unspecified" CACHE STRING "Build name variable for CTest")
set(SITE "Unknown" CACHE STRING "Site name for CTest")
add_test ( get_id command uname -a)

