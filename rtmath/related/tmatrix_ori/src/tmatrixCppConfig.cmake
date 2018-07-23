# - Config file for the Tmatrix package
# It defines the following variables
#  TMATRIX_CPP_INCLUDE_DIRS - include directories
#  TMATRIX_CPP_LIBRARIES    - libraries to link against
#  TMATRIX_CPP_EXECUTABLE   - an executable
 
# Compute paths
get_filename_component(TMATRIX_CPP_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(TMATRIX_CPP_INCLUDE_DIRS "${TMATRIX_CPP_CMAKE_DIR}/../include")
set(TMATRIX_INCLUDE_DIRS "${TMATRIX_CMAKE_DIR}/../include")
 
# Our library dependencies (contains definitions for IMPORTED targets)
include("${TMATRIX_CPP_CMAKE_DIR}/tmatrixCpp.cmake")
 
# These are IMPORTED targets
set(TMATRIX_CPP_LIBRARIES tmatrix-cpp )
#set(TMATRIX_CPP_EXECUTABLE bar)

# tmatrix-fortran should already be loaded.
set(TMATRIX_LIBRARIES tmatrix-cpp tmatrix-fortran )

