# - Config file for the Tmatrix package
# It defines the following variables
#  TMATRIX_RANDOM_FORTRAN_INCLUDE_DIRS - include directories
#  TMATRIX_RANDOM_FORTRAN_LIBRARIES    - libraries to link against
#  TMATRIX_RANDOM_FORTRAN_EXECUTABLE   - an executable
 
# Compute paths
get_filename_component(TMATRIX_RANDOM_FORTRAN_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(TMATRIX_RANDOM_FORTRAN_INCLUDE_DIRS "${TMATRIX_RANDOM_FORTRAN_CMAKE_DIR}/../include")
 
# Our library dependencies (contains definitions for IMPORTED targets)
include("${TMATRIX_RANDOM_FORTRAN_CMAKE_DIR}/tmatrixRandomFortran.cmake")
 
# These are IMPORTED targets 
set(TMATRIX_RANDOM_FORTRAN_LIBRARIES tmatrix_random-fortran)
#set(TMATRIX_RANDOM_EXECUTABLE bar)

