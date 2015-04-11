# - Find Voro++

if (VORO_INCLUDE_DIR AND VORO_LIBRARIES)
  set(VORO_FIND_QUIETLY TRUE)
endif()

# Include dir
find_path(VORO_INCLUDE_DIR 
	NAMES voro++.hh
	PATH_SUFFIXES voro++ Voro voro
)

# Library
find_library(VORO_LIBRARY 
  NAMES libvoro++.so
)

# handle the QUIETLY and REQUIRED arguments and set POSTGRESQL_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(VORO DEFAULT_MSG VORO_LIBRARY VORO_INCLUDE_DIR)

IF(VORO_FOUND)
  SET( VORO_LIBRARIES ${VORO_LIBRARY} )
ELSE(VORO_FOUND)
  SET( VORO_LIBRARIES )
ENDIF(VORO_FOUND)

# Lastly make it so that the POSTGRESQL_LIBRARY and VORO_INCLUDE_DIR variables
# only show up under the advanced options in the gui cmake applications.
MARK_AS_ADVANCED( VORO_LIBRARY VORO_INCLUDE_DIR )

