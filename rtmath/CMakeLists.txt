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

include(common)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Ask for some options
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for headers")
if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/CMake/rtmath)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
	"Installation directory for CMake files")
 
set(MAJOR 0)
set(MINOR 2)

# Get subversion revision information - for use in cpack
#EXECUTE_PROCESS("svn info . | grep Revision: |cut -c11-" 
#	OUTPUT_VARIABLE REVISION)
EXECUTE_PROCESS(COMMAND "svn" "info" "${CMAKE_CURRENT_SOURCE_DIR}"
	OUTPUT_FILE "SVN-INFO")

EXECUTE_PROCESS(COMMAND "grep" "Revis" "SVN-INFO"
	OUTPUT_FILE "SVN-REV")

EXECUTE_PROCESS(COMMAND "cut" "-c11-" "SVN-REV"
	OUTPUT_VARIABLE REVISION
	OUTPUT_STRIP_TRAILING_WHITESPACE)
	
set(RTMATH_VERSION
	${MAJOR}.${MINOR}.${REVISION} )



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
SET( ENV_MOD_DIR_PREFIX
        ${CMAKE_INSTALL_PREFIX}/etc/modulefiles
        CACHE STRING
        "Environment-modules package file install directory"
)

option ( INSTALL_DATA
	"Install data directory" OFF)
option ( INSTALL_MODULES
	"Install environment-modules file for easy pathing"
	${USE_MODULES})

set(MODULES_FILENAME
	rtmath
	CACHE STRING
	"Name of the environment module"
	)

option ( USE_OPENMP
	"Enable OpenMP to speed up execution in certain areas" OFF)

option ( BUILD_APPS
	"Build the default applications" OFF)
option ( RTMATH_OLD_QT
	"Set this flag if an older version of Qt4 is present. Ensures compatibility" OFF)

# Find the packages for the core libraries. Applications do their own supplemental package finding.
#	find_package(ROOT REQUIRED)
#	find_package(NetCDF REQUIRED)
#	find_package(PostgreSQL)
#	find_package(PQXX)
#	find_package(Boost REQUIRED)
#find_package(OpenMP)
# Qt4 is loaded only of make_guis is set
#find_package(Qt4 REQUIRED)
#find_package(Curses REQUIRED)
# Compression for serialization. It even detects which are present.
#	find_package(ZLIB)
#	find_package(BZip2)
# PCL is needed for convex and concave hull calculations.
# Used by the core library for statistics generation.
#	find_package(Qhull REQUIRED)
#	find_package(VTK REQUIRED)
#	find_package(PCL 1.0 REQUIRED)
# ImageMagick is needed for loading PNG files
#	find_package(ImageMagick COMPONENTS Magick++ REQUIRED)

#	find_package(tmatrix)
#	find_package(rycurses)

# Take OpenMP option and enable / disable as appropriate
IF(USE_OPENMP AND OPENMP_FOUND)
	SET (COMMON_CFLAGS ${COMMON_CFLAGS} ${OpenMP_CXX_FLAGS})
ENDIF()

# Enable C++11
# g++
IF(${CMAKE_COMPILER_IS_GNUCXX})
	if ("${COMMON_CFLAGS}" MATCHES "0x")
	else()
		SET (COMMON_CFLAGS ${COMMON_CFLAGS} -std=c++0x)
	endif()
ENDIF()

IF(MSVC)
	# MSVC parallel builds by default
	if ("${COMMON_CFLAGS}" MATCHES "MP")
	else()
		SET(COMMON_CFLAGS ${COMMON_CFLAGS} /MP)
	endif()
	add_definitions(-DCONF="$(Configuration)")
else()
	add_definitions(-DCONF="${CMAKE_BUILD_TYPE}")
ENDIF()


# The target processing
add_definitions(-DWITH_CMAKE)
# Recurse into the subdirectories
# add_subdirectory (deps)
#add_subdirectory (rtmath-libs/rtmath-mie)
add_subdirectory (rtmath)
if (BUILD_APPS)
	add_subdirectory (rtmath-apps)
endif()


# Beginning of CPack stuff


# Add in packaging system support

set(CPACK_PACKAGE_NAME "rtmath")
set(CPACK_PACKAGE_VENDOR "Ryan Honeyager")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "rtmath - radiative transfer math library")
set(CPACK_PACKAGE_VERSION "${RTMATH_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH ${REVISION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "rtmath")
set(CPACK_PACKAGE_CONTACT "Ryan Honeyager (rhoneyager@fsu.edu)")

set (CPACK_COMPONENTS_ALL 
	Applications
	Data
	Libraries 
	Tests 
	Headers
)

set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION 
	"All of the rtmath apps")
set(CPACK_COMPONENT_DATA_DESCRIPTION 
	"Basic configuration files")
set(CPACK_COMPONENT_LIBRARIES_DESCRIPTION 
	"The compiled libraries")
set(CPACK_COMPONENT_TESTS_DESCRIPTION 
	"Testing application to verify that the library is working properly.")
set(CPACK_COMPONENT_HEADERS_DESCRIPTION 
	"Headers for code development")

set(CPACK_COMPONENT_HEADERS_DEPENDS Libraries)
set(CPACK_COMPONENT_TESTS_DEPENDS Libraries Data)
set(CPACK_COMPONENT_APPLICATIONS_DEPENDS Libraries Data)

set(CPACK_COMPONENT_LIBRARIES_REQUIRED 1)

set(CPACK_RPM_PACKAGE_REQUIRES 
	"cmake >= 2.8, root-physics, root, tcsh, postgresql-devel, boost-devel >= 1.45, boost-filesystem, gcc-c++ >= 4.6, netcdf-devel, netcdf-cxx-devel, netcdf-cxx4-devel, hdf5-devel, libpqxx-devel, pcl-devel"
	)

set(CPACK_DEBIAN_PACKAGE_DEPENDS
	"cmake (>= 2.8), root-v5.32.00, tcsh, libboost-dev (>= 1.46.1), libboost-filesystem-dev (>= 1.46.1), g++ (>= 4:4.6.1), libnetcdf-dev, libnetcdf6, libhdf5-dev, libpqxx-dev, libpq-dev"
	)

# This must always be last!
include(CPack)

