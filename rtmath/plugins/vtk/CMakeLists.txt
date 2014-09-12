include (common)
include (commonRtmathApps)
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/"
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
include(${VTK_USE_FILE})

add_library (rtmath_plugin_vtk SHARED 
	plugin-vtk.cpp
	plugin-vtk.h
	vtk-shapefile.cpp
	vtk-hulls.cpp
	)
addlib(rtmath_plugin_vtk SHARED)

target_link_libraries(rtmath_plugin_vtk ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat ${VTK_LIBRARIES})
addplugin(rtmath_plugin_vtk Shapes)

