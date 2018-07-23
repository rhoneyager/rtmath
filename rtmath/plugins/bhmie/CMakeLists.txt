include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/ryan-debug/)

add_subdirectory (../../related/bhmie "${CMAKE_CURRENT_BINARY_DIR}/rtmath_bhmie")

add_library (rtmath_plugin_bhmie SHARED 
	plugin-bhmie.cpp
	plugin-bhmie.h
	)
addlib(rtmath_plugin_bhmie SHARED)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


target_link_libraries(rtmath_plugin_bhmie ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat rtmath_bhmie)
addplugin(rtmath_plugin_bhmie non_dda rtmath)

