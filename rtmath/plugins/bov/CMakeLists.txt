include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library (rtmath_plugin_bov SHARED plugin-bov.cpp)
addlib(rtmath_plugin_bov SHARED)

set_target_properties( rtmath_plugin_bov PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_bov ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath_plugin_bov 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

