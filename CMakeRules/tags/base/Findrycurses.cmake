###############################################################################
# Find rycurses
#
# This sets the following variables:
# RYCURSES_FOUND - True if RYCURSES was found.
# RYCURSES_INCLUDE_DIRS - Directories containing the include files.
# RYCURSES_LIBRARIES - Libraries needed.
# RYCURSES_DEFINITIONS - Compiler flags for.

set(RYCURSES_CPP_RELEASE_NAME rycurses)

find_file(RYCURSES_HEADER
	NAMES rycurses/rycurses.h rycurses.h
	HINTS "${RYCURSES_ROOT}" "$ENV{RYCURSES_ROOT}" "${RYCURSES_INCLUDE_DIR}"
	PATHS "$ENV{PROGRAMFILES}/rycurses" "$ENV{PROGRAMW6432}/rycurses"
	PATH_SUFFIXES rycurses include)

set(RYCURSES_HEADER "${RYCURSES_HEADER}" CACHE INTERNAL "rycurses header" FORCE )

if(RYCURSES_HEADER)
  get_filename_component(rycurses_header ${RYCURSES_HEADER} NAME_WE)
    get_filename_component(RYCURSES_INCLUDE_DIR ${RYCURSES_HEADER} PATH)
    get_filename_component(RYCURSES_INCLUDE_DIR ${RYCURSES_INCLUDE_DIR} PATH)
else()
  set(RYCURSES_INCLUDE_DIR "RYCURSES_INCLUDE_DIR-NOTFOUND")
endif()

set(RYCURSES_INCLUDE_DIR "${RYCURSES_INCLUDE_DIR}" CACHE PATH "rycurses include dir." FORCE)

find_library(RYCURSES_CPP_LIBRARY 
             NAMES ${RYCURSES_CPP_RELEASE_NAME}
             HINTS "${RYCURSES_ROOT}" "$ENV{RYCURSES_ROOT}" "${RYCURSES_INCLUDE_DIR}/../"
             PATHS "$ENV{PROGRAMFILES}/rycurses" "$ENV{PROGRAMW6432}/rycurses"
             PATH_SUFFIXES project build bin lib)

set (RYCURSES_LIBRARIES ${RYCURSES_CPP_LIBRARY})

set(RYCURSES_INCLUDE_DIRS ${RYCURSES_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
#find_package_handle_standard_args(rycurses DEFAULT_MSG RYCURSES_CPP_LIBRARY RYCURSES_INCLUDE_DIR)
find_package_handle_standard_args(rycurses DEFAULT_MSG RYCURSES_LIBRARIES RYCURSES_INCLUDE_DIR)

mark_as_advanced(RYCURSES_CPP_LIBRARY RYCURSES_INCLUDE_DIR RYCURSES_LIBRARIES rycurses_DIR)

if(RYCURSES_FOUND)
  set(HAVE_RYCURSES ON)
  message(STATUS "rycurses found (include: ${RYCURSES_INCLUDE_DIRS}, lib: ${RYCURSES_LIBRARIES})")
endif(RYCURSES_FOUND)

