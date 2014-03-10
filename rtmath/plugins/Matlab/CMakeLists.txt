include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Matlab REQUIRED)
include_directories(${MATLAB_INCLUDE_DIR})

add_library (rtmath_plugin_Matlab SHARED 
	plugin-Matlab.cpp
	plugin-Matlab.h
	)
addlib(rtmath_plugin_Matlab SHARED)

set_target_properties( rtmath_plugin_Matlab PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_Matlab ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat ${MATLAB_LIBRARIES})
INSTALL(TARGETS rtmath_plugin_Matlab 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

