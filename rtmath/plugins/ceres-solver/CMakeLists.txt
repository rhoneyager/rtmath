include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_library (rtmath_plugin_ceres SHARED 
	plugin-ceres.cpp
	plugin-ceres.h
	)
addlib(rtmath_plugin_ceres SHARED)

set_target_properties( rtmath_plugin_ceres PROPERTIES FOLDER "Plugins/Optimization")
target_link_libraries(rtmath_plugin_ceres ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat ${CERES_LIBRARIES})
INSTALL(TARGETS rtmath_plugin_ceres 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

