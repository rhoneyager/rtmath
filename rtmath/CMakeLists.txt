# CMake solution for the rtmath library
# This conveniently can produce unix makefiles, eclipse .project files, 
# visual studio solutions and xcode projects

cmake_minimum_required (VERSION 2.6)
project(rtmath)

# Recurse into the subdirectories

add_subdirectory (rtmath)
add_subdirectory (rtmath-apps)



# Add in packaging system support

set(CPACK_PACKAGE_NAME "rtmath")
set(CPACK_PACKAGE_VENDOR "Ryan Honeyager")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "rtmath - radiative transfer math library")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "rtmath")
set(CPACK_PACKAGE_CONTACT "Ryan Honeyager (rhoneyager@fsu.edu)")

set (CPACK_COMPONENTS_ALL 
	Applications 
	Libraries 
	Tests 
	Headers
)

set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION 
	"All of the rtmath apps")
set(CPACK_COMPONENT_LIBRARIESS_DESCRIPTION 
	"The compiled libraries")
set(CPACK_COMPONENT_TESTS_DESCRIPTION 
	"Testing application to verify that the library is working properly.")
set(CPACK_COMPONENT_HEADERS_DESCRIPTION 
	"Headers for code development")

set(CPACK_COMPONENT_HEADERS_DEPENDS Libraries)
set(CPACK_COMPONENT_TESTS_DEPENDS Libraries)
set(CPACK_COMPONENT_APPLICATIONS_DEPENDS Libraries)

set(CPACK_COMPONENT_LIBRARIES_REQUIRED 1)

# TODO: add RPM building package separation
# TODO: add Ubuntu package separation
# TODO: add package dependencies
# TODO: add structure to MSVC build files (solution folders)
# TODO: add headers to rtmath solution build, in their appropriiate directories
# TODO: add postgresql support

# This must always be last!
include(CPack)

