include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
find_package(scatdb_ryan REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SCATDB_RYAN_INCLUDE_DIRS})

add_library (rtmath_plugin_scatdb_ryan SHARED 
	provider.cpp
	plugin-scatdb-ryan.cpp
	plugin-scatdb-ryan.h
	)
addlib(rtmath_plugin_scatdb_ryan SHARED)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


target_link_libraries(rtmath_plugin_scatdb_ryan ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat )
addplugin(rtmath_plugin_scatdb_ryan non_dda rtmath)

