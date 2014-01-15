include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(silo REQUIRED)
include_directories(${SILO_INCLUDE_DIRS})
#find_package(netCDF REQUIRED)
#include_directories(${NetCDF_INCLUDE_DIRS})
#find_package(HDF5 COMPONENTS C REQUIRED)
#include_directories(${HDF5_INCLUDE_DIRS})
#add_definitions(${HDF5_DEFINITIONS})
#find_package(ZLIB REQUIRED)
#include_directories(${ZLIB_INCLUDE_DIRS})
#find_package(SZIP REQUIRED)

add_library (rtmath_plugin_silo SHARED 
	plugin-silo.cpp
	silo-shapefile.cpp
	CurveMesh3d.cpp
	CurveMesh3d.h
	MaterialList.cpp
	MaterialList.h
	QuadMesh3d.cpp
	QuadMesh3d.h
	RectilinearMesh3d.cpp
	RectilinearMesh3d.h
	remake.h
	)
addlib(rtmath_plugin_silo SHARED)

set_target_properties( rtmath_plugin_silo PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_silo ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat silo)
INSTALL(TARGETS rtmath_plugin_silo 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

