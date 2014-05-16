include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Tmatrix REQUIRED)
include_directories(${TMATRIX_INCLUDE_DIRS})

add_library (rtmath_plugin_tmatrix SHARED 
	plugin-tmatrix.cpp
	plugin-tmatrix.h
	)
addlib(rtmath_plugin_tmatrix SHARED)

set_target_properties( rtmath_plugin_tmatrix PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_tmatrix ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath_plugin_tmatrix 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

