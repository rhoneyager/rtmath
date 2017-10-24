# - Find PQXX Libraries
#
# This module defines:
#  PQXX_FOUND - True if the package is found
#  PQXX_INCLUDE_DIR - containing libpq-fe.h
#  PQXX_LIBRARIES - Libraries to link to use PQ functions.

if (PQXX_INCLUDE_DIR AND PQXX_LIBRARIES)
  set(PQXX_FIND_QUIETLY TRUE)
endif (PQXX_INCLUDE_DIR AND PQXX_LIBRARIES)

# Include dir
find_path(PQXX_INCLUDE_DIR 
	NAMES pqxx
	PATH_SUFFIXES pqxx
)

# Library
find_library(PQXX_LIBRARY 
  NAMES pqxx
)

# handle the QUIETLY and REQUIRED arguments and set POSTGRESQL_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PQXX DEFAULT_MSG PQXX_LIBRARY PQXX_INCLUDE_DIR)

IF(PQXX_FOUND)
  SET( PQXX_LIBRARIES ${PQXX_LIBRARY} )
ELSE(PQXX_FOUND)
  SET( PQXX_LIBRARIES )
ENDIF(PQXX_FOUND)

# Lastly make it so that the POSTGRESQL_LIBRARY and PQXX_INCLUDE_DIR variables
# only show up under the advanced options in the gui cmake applications.
MARK_AS_ADVANCED( PQXX_LIBRARY PQXX_INCLUDE_DIR )

