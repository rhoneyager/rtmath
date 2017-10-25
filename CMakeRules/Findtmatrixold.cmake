###############################################################################
# Find tmatrix
#
# This sets the following variables:
# TMATRIX_FOUND - True if TMATRIX was found.
# TMATRIX_INCLUDE_DIRS - Directories containing the include files.
# TMATRIX_LIBRARIES - Libraries needed.
# TMATRIX_DEFINITIONS - Compiler flags for.

set(TMATRIX_CPP_RELEASE_NAME tmatrix-cpp)
set(TMATRIX_FORTRAN_RELEASE_NAME tmatrix-fortran)

find_file(TMATRIX_HEADER
	NAMES tmatrix/tmatrix.h tmatrix.h
	HINTS "${TMATRIX_ROOT}" "$ENV{TMATRIX_ROOT}" "${TMATRIX_INCLUDE_DIR}"
	PATHS "$ENV{PROGRAMFILES}/tmatrix" "$ENV{PROGRAMW6432}/tmatrix"
	PATH_SUFFIXES tmatrix include)

set(TMATRIX_HEADER "${TMATRIX_HEADER}" CACHE INTERNAL "tmatrix header" FORCE )

if(TMATRIX_HEADER)
  get_filename_component(tmatrix_header ${TMATRIX_HEADER} NAME_WE)
    get_filename_component(TMATRIX_INCLUDE_DIR ${TMATRIX_HEADER} PATH)
    get_filename_component(TMATRIX_INCLUDE_DIR ${TMATRIX_INCLUDE_DIR} PATH)
else()
  set(TMATRIX_INCLUDE_DIR "TMATRIX_INCLUDE_DIR-NOTFOUND")
endif()

set(TMATRIX_INCLUDE_DIR "${TMATRIX_INCLUDE_DIR}" CACHE PATH "tmatrix include dir." FORCE)

find_library(TMATRIX_CPP_LIBRARY 
             NAMES ${TMATRIX_CPP_RELEASE_NAME}
             HINTS "${TMATRIX_ROOT}" "$ENV{TMATRIX_ROOT}" "${TMATRIX_INCLUDE_DIR}/../"
             PATHS "$ENV{PROGRAMFILES}/tmatrix" "$ENV{PROGRAMW6432}/tmatrix" 
             PATH_SUFFIXES project build bin lib)

find_library(TMATRIX_FORTRAN_LIBRARY 
             NAMES ${TMATRIX_FORTRAN_RELEASE_NAME}
             HINTS "${TMATRIX_ROOT}" "$ENV{TMATRIX_ROOT}" "${TMATRIX_INCLUDE_DIR}/../"
             PATHS "$ENV{PROGRAMFILES}/tmatrix" "$ENV{PROGRAMW6432}/tmatrix" 
             PATH_SUFFIXES project build bin lib)

if(NOT TMATRIX_FORTRAN_LIBRARY)
	set (TMATRIX_LIBRARIES ${TMATRIX_CPP_LIBRARY})
else()
	set (TMATRIX_LIBRARIES ${TMATRIX_CPP_LIBRARY} ${TMATRIX_FORTRAN_LIBRARY})
endif()

set(TMATRIX_INCLUDE_DIRS ${TMATRIX_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(tmatrix DEFAULT_MSG TMATRIX_CPP_LIBRARY TMATRIX_INCLUDE_DIR)
find_package_handle_standard_args(tmatrix DEFAULT_MSG TMATRIX_LIBRARIES TMATRIX_INCLUDE_DIR)

mark_as_advanced(TMATRIX_CPP_LIBRARY TMATRIX_FORTRAN_LIBRARY TMATRIX_INCLUDE_DIR TMATRIX_LIBRARIES tmatrix_DIR)

if(TMATRIX_FOUND)
  set(HAVE_TMATRIX ON)
  message(STATUS "tmatrix found (include: ${TMATRIX_INCLUDE_DIRS}, lib: ${TMATRIX_LIBRARIES})")
endif(TMATRIX_FOUND)

