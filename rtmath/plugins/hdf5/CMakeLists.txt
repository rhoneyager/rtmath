include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(HDF5 COMPONENTS C CXX REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR_CPP})
add_definitions(${HDF5_DEFINITIONS})

if(DEFINED HDF5_C_LIBRARIES)
	set (h5libs ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
else()
	set (h5libs hdf5 hdf5_cpp)
endif()
#FIND_PACKAGE(szip)
#IF (DEFINED SZIP_LIBRARIES)
#	include_directories(${SZIP_INCLUDE_DIRS})
#	SET (h5libs ${h5libs} szip)
#ENDIF()

add_library (rtmath_plugin_hdf5 SHARED 
	plugin-hdf5.cpp
	plugin-hdf5.h
	hdf5-ddoutput.cpp
	hdf5-shapefile.cpp
	hdf5-shapestats.cpp
	hdf5-voronoi.cpp
	)
addlib(rtmath_plugin_hdf5 SHARED)

set_target_properties( rtmath_plugin_hdf5 PROPERTIES FOLDER "Plugins/ddscat")
target_link_libraries(rtmath_plugin_hdf5 
	${rtmath_ddscat-libs} rtmath_core rtmath_ddscat
	${h5libs}
	)
INSTALL(TARGETS rtmath_plugin_hdf5 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

