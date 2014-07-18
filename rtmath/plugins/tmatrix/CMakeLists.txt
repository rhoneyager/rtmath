include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Tmatrix REQUIRED)
include_directories(${TMATRIX_INCLUDE_DIRS})

add_library (rtmath_plugin_tmatrix SHARED 
	plugin-tmatrix.cpp
	plugin-tmatrix.h
	)
addlib(rtmath_plugin_tmatrix SHARED)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


target_link_libraries(rtmath_plugin_tmatrix ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addplugin(rtmath_plugin_tmatrix non_dda)

