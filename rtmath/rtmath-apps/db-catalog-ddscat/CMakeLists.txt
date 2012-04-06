include (common)

# For now, don't build on windows
IF(${UNIX})
add_executable (rtmath-db-catalog-ddscat process_ddscat.cpp)
addapp(rtmath-db-catalog-ddscat)
ENDIF(${UNIX})

