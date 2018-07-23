# - Config file for the Tmatrix package
# It defines the following variables
#  TMATRIX_FORTRAN_INCLUDE_DIRS - include directories
#  TMATRIX_FORTRAN_LIBRARIES    - libraries to link against
#  TMATRIX_FORTRAN_EXECUTABLE   - an executable
 
# Compute paths
get_filename_component(TMATRIX_FORTRAN_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(TMATRIX_FORTRAN_INCLUDE_DIRS "${TMATRIX_FORTRAN_CMAKE_DIR}/../include")
 
# Our library dependencies (contains definitions for IMPORTED targets)
include("${TMATRIX_FORTRAN_CMAKE_DIR}/tmatrixFortran.cmake")
 
# These are IMPORTED targets 
set(TMATRIX_FORTRAN_LIBRARIES tmatrix-fortran)
#set(TMATRIX_EXECUTABLE bar)

