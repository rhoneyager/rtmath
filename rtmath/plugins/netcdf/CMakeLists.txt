include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(netcdf REQUIRED)
#message("Include ${netcdf_INCLUDE_DIRS}")
include_directories(${netcdf_INCLUDE_DIRS})

add_library (rtmath_plugin_netcdf SHARED 
	plugin-netcdf.cpp
	plugin-netcdf.h
	)
addlib(rtmath_plugin_netcdf SHARED)

set_target_properties( rtmath_plugin_netcdf PROPERTIES FOLDER "Plugins/RT")
target_link_libraries(rtmath_plugin_netcdf 
	${rtmath_data-libs} 
	rtmath_core 
	rtmath_voronoi
	rtmath_data
	netcdf)
INSTALL(TARGETS rtmath_plugin_netcdf 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

