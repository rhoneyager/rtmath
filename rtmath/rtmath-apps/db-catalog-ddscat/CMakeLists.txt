# Create the test program and link it with the rtmath library

include (../../CMakeRules/common.cmake)

# For now, don't build on windows
IF(${UNIX})
add_executable (rtmath-db-catalog-ddscat process_ddscat.cpp)
set(appname rtmath-db-catalog-ddscat)
include (../../CMakeRules/addapp.cmake)
ENDIF(${UNIX})

