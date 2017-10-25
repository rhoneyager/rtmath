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

addBoostUniform(program_options )
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

if(NOT ZLIB_INCLUDE_DIR)
	if (DEFINED ENV{ZLIB_ROOT})
		set(ZLIB_ROOT $ENV{ZLIB_ROOT})
	ENDIF()
endif()

add_subdirectory (rtmath/rtmath)
add_subdirectory (rtmath/plugins)
if (BUILD_APPS)
	add_subdirectory (rtmath/rtmath-apps)
endif()


adddocs()

set(BUILDNAME "Unspecified" CACHE STRING "Build name variable for CTest")
set(SITE "Unknown" CACHE STRING "Site name for CTest")
add_test ( get_id command uname -a)

