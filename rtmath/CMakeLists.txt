# CMake solution for the rtmath library
# This conveniently can produce unix makefiles, eclipse .project files, 
# visual studio solutions and xcode projects

cmake_minimum_required (VERSION 2.8)
#project(rtmath CXX Fortran C)
project(rtmath CXX C)
include (CTest)
enable_testing()

# Add path for custom modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

include(SVNinfo)
include(addBaseProject)
include(addlib)
include(addAssembly)
include(addconfiginfo)
include(addInstallDirs)
include(addModuleFiles)
include(addBoostUniform)
include(adddocs)
include(generateAssemblyGuid)

 
set(MAJOR 0)
set(MINOR 3)
set(REVISION 0)
set(RTMATH_VERSION
	${MAJOR}.${MINOR}.${REVISION} )


addBaseProject()
addInstallDirs(rtmath ${RTMATH_VERSION})
addModuleFiles(rtmath ${RTMATH_VERSION})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(REGISTRY_PLUGINS_OVERRIDE_DIR ../plugins CACHE PATH "Additional plugin search directory (leave to ../plugins for none)")


if(WIN32 AND NOT CYGWIN)
	set( DATA_DIR_PREFIX_CAND
		C:/rtmath )
	set(USE_MODULES OFF)
else()
	set( DATA_DIR_PREFIX_CAND
		/lab/rhoneyager/rtmath )
	set(USE_MODULES ON)
endif()

SET( DATA_DIR_PREFIX
	${DATA_DIR_PREFIX_CAND}
	CACHE STRING
	"Data directory prefix for basic configuration"
)

option ( INSTALL_DATA
	"Install data directory" OFF)
option ( USE_OPENMP
	"Enable OpenMP to speed up execution in certain areas" OFF)
option ( BUILD_APPS
	"Build the default applications" OFF)
option ( RTMATH_OLD_QT
	"Set this flag if an older version of Qt4 is present. Ensures compatibility" OFF)

# Take OpenMP option and enable / disable as appropriate
IF(USE_OPENMP AND OPENMP_FOUND)
	SET (COMMON_CFLAGS ${COMMON_CFLAGS} ${OpenMP_CXX_FLAGS})
ENDIF()
# Enable C++11
# g++
IF(CMAKE_COMPILER_IS_GNUCXX)
	SET (COMMON_CFLAGS ${COMMON_CFLAGS} -std=c++11)
ENDIF()
IF(MSVC)
	# MSVC parallel builds by default
	if ("${COMMON_CFLAGS}" MATCHES "MP")
	else()
		SET(COMMON_CFLAGS ${COMMON_CFLAGS} /MP)
	endif()
ENDIF()


# The target processing
add_definitions(-DWITH_CMAKE)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Set a uniform output directory (to make app debugs easier)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_Debug ${CMAKE_BINARY_DIR}/Debug)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_Release ${CMAKE_BINARY_DIR}/Release)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_MinSizeRel ${CMAKE_BINARY_DIR}/MinSizeRel)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RelWithDebInfo ${CMAKE_BINARY_DIR}/RelWithDebInfo)

# Recurse into the subdirectories
# add_subdirectory (deps)
#add_subdirectory (rtmath-libs/rtmath-mie)
add_subdirectory (rtmath)
add_subdirectory (plugins)
if (BUILD_APPS)
	add_subdirectory (rtmath-apps)
endif()


adddocs()

