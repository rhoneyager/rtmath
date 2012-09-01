# CMake solution for the rtmath library
# This conveniently can produce unix makefiles, eclipse .project files, 
# visual studio solutions and xcode projects

cmake_minimum_required (VERSION 2.8)
project(rtmath CXX Fortran C)

include (CTest)
enable_testing()

# Add path for custom modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)
#message("Modules " ${CMAKE_MODULE_PATH})

include(common)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if( CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR AND MSVC_IDE )
  message(FATAL_ERROR "In-source builds are not allowed.
CMake would overwrite the VS project files. 
Please create a directory and run cmake from there, passing the path
to this source directory as the last argument.
This process created the file `CMakeCache.txt' and the directory `CMakeFiles'.
Please delete them.")
endif()

# Ask for some options
if(WIN32)
	set( DATA_DIR_PREFIX_CAND
		C:/rtmath )
else()
	set( DATA_DIR_PREFIX_CAND
		/data/rhoneyag/rtmath )
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
set(LIB_SUFFIX "" CACHE STRING "Define suffix of lib directory name (32/64)" )


option ( INSTALL_DATA
	"Install data directory" OFF)
option ( INSTALL_MODULES
	"Install environment-modules file for easy pathing"
	OFF)
set(MODULES_FILENAME
	rtmath
	CACHE STRING
	"Name of the environment module"
	)

option ( USE_OPENMP
	"Enable OpenMP to speed up execution in certain areas" OFF)
if(MSVC)
	set(LOAD_PACKAGES_DEFAULT OFF)
else()
	set(LOAD_PACKAGES_DEFAULT ON)
endif()
option ( LOAD_PACKAGES
	"Master switch to enable / disable package finding." 
	${LOAD_PACKAGES_DEFAULT}
)
option ( BUILD_APPS
	"Build the default applications" ON)
option ( COMPRESSION_ENABLED 
	"Enable compression" ON)

if(LOAD_PACKAGES)
	# Find the locations of ROOT, OpenMP and others
	find_package(ROOT REQUIRED)
	find_package(NetCDF REQUIRED)
	find_package(PostgreSQL)
	find_package(PQXX)
	find_package(Boost REQUIRED)
	find_package(OpenMP)
	# Qt4 is loaded only of make_guis is set
	#find_package(Qt4 REQUIRED)
	#find_package(Curses REQUIRED)
	# Compression for serialization. It even detects which are present.
	find_package(ZLIB)
	find_package(BZip2)
	# PCL is needed for convex and concave hull calculations.
	# Used by the core library for statistics generation.
	find_package(VTK REQUIRED)
	find_package(PCL 1.0 REQUIRED)
	# ImageMagick is needed for loading PNG files
	find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
endif()
# Take OpenMP option and enable / disable as appropriate
IF(USE_OPENMP)
	IF (${OPENMP_FOUND})
		SET (COMMON_CFLAGS ${COMMON_CFLAGS} ${OpenMP_CXX_FLAGS})
	ENDIF()
ENDIF()

# Set available compression schemes
if(LOAD_PACKAGES)
	set(COMPRESS_ZLIB 0)
	set(COMPRESS_GZIP 0)
	set(COMPRESS_BZIP2 0)
	if (COMPRESSION_ENABLED)
		if (ZLIB_FOUND)
			set(COMPRESS_ZLIB 1)
			set(COMPRESS_GZIP 1)
		endif()
	# BZIP2_FOUND requires that the debug libary be found, which is not desired.
		if (BZIP2_INCLUDE_DIR)
			set(COMPRESS_BZIP2 1)
		endif()
	endif()

else()
	if (COMPRESSION_ENABLED)
		set(COMPRESS_ZLIB 1)
		set(COMPRESS_GZIP 1)
		set(COMPRESS_BZIP2 1)
	endif()
endif()

# Set basic common libraries
#SET ( COMMON_LIBS 
#	boost_filesystem
#	boost_date_time
#	boost_unit_test_framework
#	boost_system
#)
IF(${MSVC})
ELSE()
	SET ( COMMON_LIBS ${COMMON_LIBS} 
		m
		boost_filesystem
		boost_date_time
		boost_iostreams
		boost_program_options
		boost_serialization
		boost_unit_test_framework
		boost_system
	)
	#SET (	COMMON_LIBS ${COMMON_LIBS} netcdf m boost_filesystem
	#	pqxx boost_date_time pq
	#	boost_unit_test_framework boost_system ${ROOT_LIBRARIES})
ENDIF()

# Enable C++11
# g++
IF(${CMAKE_COMPILER_IS_GNUCXX})
	if ("${COMMON_CFLAGS}" MATCHES "0x")
	else()
		SET (COMMON_CFLAGS ${COMMON_CFLAGS} -std=c++0x)
	endif()
ENDIF()

IF(${MSVC})
	# MSVC parallel builds by defaul
	if ("${COMMON_CFLAGS}" MATCHES "MP")
	else()
		SET(COMMON_CFLAGS ${COMMON_CFLAGS} /MP)
	endif()
ENDIF(${MSVC})


# The target processing
# Recurse into the subdirectories
add_subdirectory (deps)
add_subdirectory (rtmath)
if (BUILD_APPS)
	add_subdirectory (rtmath-apps)
endif()


# Beginning of CPack stuff


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
	

# Add in packaging system support

set(CPACK_PACKAGE_NAME "rtmath")
set(CPACK_PACKAGE_VENDOR "Ryan Honeyager")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "rtmath - radiative transfer math library")
set(CPACK_PACKAGE_VERSION "0.1.${REVISION}")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
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

# TODO: add RPM building package separation
# TODO: add Ubuntu package separation
# TODO: add structure to MSVC build files (solution folders)
# TODO: add headers to rtmath solution build, in their appropriiate directories
# TODO: add postgresql support
set(CPACK_RPM_PACKAGE_REQUIRES 
	"cmake >= 2.8, root-physics, root, tcsh, postgresql-devel, boost-devel >= 1.45, boost-filesystem, gcc-c++ >= 4.6, netcdf-devel, netcdf-cxx-devel, netcdf-cxx4-devel, hdf5-devel, libpqxx-devel, pcl-devel"
	)

set(CPACK_DEBIAN_PACKAGE_DEPENDS
	"cmake (>= 2.8), root-v5.32.00, tcsh, libboost-dev (>= 1.46.1), libboost-filesystem-dev (>= 1.46.1), g++ (>= 4:4.6.1), libnetcdf-dev, libnetcdf6, libhdf5-dev, libpqxx-dev, libpq-dev"
	)

# This must always be last!
include(CPack)

