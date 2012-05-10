# CMake solution for the rtmath library
# This conveniently can produce unix makefiles, eclipse .project files, 
# visual studio solutions and xcode projects

cmake_minimum_required (VERSION 2.6)
project(rtmath)

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

# Recurse into the subdirectories
add_subdirectory (rtmath)
add_subdirectory (rtmath-apps)

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
# TODO: add package dependencies
# TODO: add structure to MSVC build files (solution folders)
# TODO: add headers to rtmath solution build, in their appropriiate directories
# TODO: add postgresql support
set(CPACK_RPM_PACKAGE_REQUIRES 
	"cmake >= 2.8, root-physics, root, tcsh, postgresql-devel, boost-devel >= 1.45, boost-filesystem, gcc-c++ >= 4.6, netcdf-devel, netcdf-cxx-devel, netcdf-cxx4-devel, hdf5-devel"
	)
#add_package_dependency(rtmath DEPENDS_ON HDF5)
#add_package_dependency(rtmath DEPENDS_ON NetCDF)

# This must always be last!
include(CPack)

