include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/ryan-debug/)
add_subdirectory (../../related/random "${CMAKE_CURRENT_BINARY_DIR}/tmm-random")

#find_package(TmatrixRandom REQUIRED)
#include_directories(${TMATRIX_RANDOM_INCLUDE_DIRS})

add_library (rtmath_plugin_tmatrix_random SHARED 
	plugin-tmatrix-random.cpp
	plugin-tmatrix-random.h
	)
addlib(rtmath_plugin_tmatrix_random SHARED)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


target_link_libraries(rtmath_plugin_tmatrix_random tmatrix_random_cpp tmatrix_random_fortran ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addplugin(rtmath_plugin_tmatrix_random non_dda rtmath)

