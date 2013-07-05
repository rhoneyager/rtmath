#include (common)

include_directories (
	${CMAKE_BINARY_DIR}
)

add_executable (rtmath-tmatrix-optimize-with-ddscat
	dataset.cpp
	dataset.h
	run.cpp
	run.h
	main.cpp 
	)

# Performs tmatrix computations
find_package(tmatrixFortran REQUIRED)
find_package(tmatrixCpp REQUIRED)
# Least-squares with Ceres Solver
#find_package(Ceres REQUIRED)
target_link_libraries( rtmath-tmatrix-optimize-with-ddscat 
	rtmath_core 
	rtmath_mie 
	${rtmath_core-libs}
	${rtmath_mie-libs}
	tmatrix-cpp
	tmatrix-fortran
	)
if(WIN32 AND NOT CYGWIN)
	# Ceres-solver has only static linkage on Windows
	#TARGET_LINK_LIBRARIES(rtmath-tmatrix-optimize-with-ddscat ${CERES_LIBRARIES})
else()
	#	TARGET_LINK_LIBRARIES(rtmath-tmatrix-optimize-with-ddscat ${CERES_LIBRARIES_SHARED})
endif()
set_target_properties( rtmath-tmatrix-optimize-with-ddscat PROPERTIES FOLDER "Apps")
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${TMATRIX_CPP_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${CERES_INCLUDES})
INSTALL(TARGETS rtmath-tmatrix-optimize-with-ddscat RUNTIME DESTINATION bin COMPONENT Applications)

