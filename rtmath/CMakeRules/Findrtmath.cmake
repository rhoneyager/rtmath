###############################################################################
# Find rtmath
#
# This sets the following variables:
# RTMATH_FOUND - True if RTMATH was found.
# RTMATH_INCLUDE_DIRS - Directories containing the include files.
# RTMATH_LIBRARIES - Libraries needed.
# RTMATH_DEFINITIONS - Compiler flags for.

set(RTMATH_CPP_RELEASE_NAME rtmath)

find_file(RTMATH_HEADER
	NAMES rtmath/rtmath.h rtmath.h
	HINTS "${RTMATH_ROOT}" "$ENV{RTMATH_ROOT}" "${RTMATH_INCLUDE_DIR}"
	PATHS "$ENV{PROGRAMFILES}/rtmath" "$ENV{PROGRAMW6432}/rtmath"
	PATH_SUFFIXES rtmath include)

set(RTMATH_HEADER "${RTMATH_HEADER}" CACHE INTERNAL "rtmath header" FORCE )

if(RTMATH_HEADER)
  get_filename_component(rtmath_header ${RTMATH_HEADER} NAME_WE)
    get_filename_component(RTMATH_INCLUDE_DIR ${RTMATH_HEADER} PATH)
    get_filename_component(RTMATH_INCLUDE_DIR ${RTMATH_INCLUDE_DIR} PATH)
else()
  set(RTMATH_INCLUDE_DIR "RTMATH_INCLUDE_DIR-NOTFOUND")
endif()

set(RTMATH_INCLUDE_DIR "${RTMATH_INCLUDE_DIR}" CACHE PATH "rtmath include dir." FORCE)

find_library(RTMATH_CPP_LIBRARY 
             NAMES ${RTMATH_CPP_RELEASE_NAME}
             HINTS "${RTMATH_ROOT}" "$ENV{RTMATH_ROOT}" "${RTMATH_INCLUDE_DIR}/../"
             PATHS "$ENV{PROGRAMFILES}/rtmath" "$ENV{PROGRAMW6432}/rtmath"
             PATH_SUFFIXES project build bin lib)

set (RTMATH_LIBRARIES ${RTMATH_CPP_LIBRARY})

set(RTMATH_INCLUDE_DIRS ${RTMATH_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(rtmath DEFAULT_MSG RTMATH_CPP_LIBRARY RTMATH_INCLUDE_DIR)
find_package_handle_standard_args(rtmath DEFAULT_MSG RTMATH_LIBRARIES RTMATH_INCLUDE_DIR)

mark_as_advanced(RTMATH_CPP_LIBRARY RTMATH_INCLUDE_DIR RTMATH_LIBRARIES rtmath_DIR)

if(RTMATH_FOUND)
  set(HAVE_RTMATH ON)
  message(STATUS "rtmath found (include: ${RTMATH_INCLUDE_DIRS}, lib: ${RTMATH_LIBRARIES})")
endif(RTMATH_FOUND)

