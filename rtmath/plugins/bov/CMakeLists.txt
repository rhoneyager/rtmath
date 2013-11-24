include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library (rtmath-plugin-bov SHARED plugin-bov.cpp)
addlib(rtmath-plugin-bov SHARED)

set_target_properties( rtmath-plugin-bov PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath-plugin-bov ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-plugin-bov 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

