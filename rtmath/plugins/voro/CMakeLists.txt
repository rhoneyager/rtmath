include (common)
include (commonRtmathApps)
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/"
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Voro REQUIRED)
include_directories(${VORO_INCLUDE_DIRS})

add_library (rtmath_plugin_voro SHARED 
	plugin-voro.cpp
	plugin-voro.h
	voro-CachedVoronoi.cpp
	voro-CachedVoronoi.h
	voro-Voronoi.cpp
	voro-Voronoi.h
	)
addlib(rtmath_plugin_voro SHARED)

target_link_libraries(rtmath_plugin_voro ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat ${VORO_LIBRARIES})
addplugin(rtmath_plugin_voro Shapes)

