include (common)
include (commonRtmathApps)
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/"
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Voro2d REQUIRED)
include_directories(${VORO2D_INCLUDE_DIRS})

add_library (rtmath_plugin_voro2d SHARED 
	plugin-voro2d.cpp
	plugin-voro2d.h
	voro2d-CachedVoronoi.cpp
	voro2d-CachedVoronoi.h
	voro2d-Voronoi.cpp
	voro2d-Voronoi.h
	)
addlib(rtmath_plugin_voro2d SHARED)

target_link_libraries(rtmath_plugin_voro2d ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat ${VORO2D_LIBRARIES})
addplugin(rtmath_plugin_voro2d Shapes)

