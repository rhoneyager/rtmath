# - Config file for the Tmatrix package
# It defines the following variables
#  TMATRIX_RANDOM_CPP_INCLUDE_DIRS - include directories
#  TMATRIX_RANDOM_CPP_LIBRARIES    - libraries to link against
#  TMATRIX_RANDOM_CPP_EXECUTABLE   - an executable
 
# Compute paths
get_filename_component(TMATRIX_RANDOM_CPP_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(TMATRIX_RANDOM_CPP_INCLUDE_DIRS "${TMATRIX_RANDOM_CPP_CMAKE_DIR}/../include")
set(TMATRIX_RANDOM_INCLUDE_DIRS "${TMATRIX_RANDOM_CMAKE_DIR}/../include")
 
# Our library dependencies (contains definitions for IMPORTED targets)
include("${TMATRIX_RANDOM_CPP_CMAKE_DIR}/tmatrixRandomCpp.cmake")
 
# These are IMPORTED targets
set(TMATRIX_RANDOM_CPP_LIBRARIES tmatrix_random-cpp )
#set(TMATRIX_RANDOM_CPP_EXECUTABLE bar)

# tmatrix-fortran should already be loaded.
set(TMATRIX_RANDOM_LIBRARIES tmatrix_random-cpp tmatrix_random-fortran )

