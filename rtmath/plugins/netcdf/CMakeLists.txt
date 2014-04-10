include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(HDF5 REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR_CPP})
add_definitions(${HDF5_DEFINITIONS})

if(DEFINED HDF5_C_LIBRARIES)
	set (h5libs ${HDF5_LIBRARIES})
else()
	set (h5libs hdf5 hdf5_cpp hdf5_f90cstub)
endif()
FIND_PACKAGE(SZIP)
#	message("${SZIP_INCLUDE_DIRS}")
#	message("${SZIP_LIBRARIES}")
IF (DEFINED SZIP_LIBRARIES)
	include_directories(${SZIP_INCLUDE_DIRS})
		SET (h5libs ${h5libs} ${SZIP_LIBRARIES})
ENDIF()
find_package(netcdf REQUIRED)
#message("Include ${netcdf_INCLUDE_DIRS}")
include_directories(${netcdf_INCLUDE_DIRS})

add_library (rtmath_plugin_netcdf SHARED 
	nc-arm_info.cpp
	nc-sacr.cpp
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

