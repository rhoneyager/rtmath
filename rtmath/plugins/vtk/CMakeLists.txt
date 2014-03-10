include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
include(${VTK_USE_FILE})

add_library (rtmath_plugin_vtk SHARED 
	plugin-vtk.cpp
	plugin-vtk.h
	)
addlib(rtmath_plugin_vtk SHARED)

set_target_properties( rtmath_plugin_vtk PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_vtk ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath_plugin_vtk 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

