include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(silo REQUIRED)
include_directories(${SILO_INCLUDE_DIRS})
find_package(HDF5 COMPONENTS C CXX REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR_CPP})
add_definitions(${HDF5_DEFINITIONS})

if(DEFINED HDF5_C_LIBRARIES)
	set (h5libs ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
else()
	set (h5libs hdf5 hdf5_cpp)
endif()
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")

add_subdirectory (../../related/rtmath_silo_cpp "${CMAKE_CURRENT_BINARY_DIR}/rtmath_silo_cpp")


add_library (rtmath_plugin_silo SHARED 
	plugin-silo.cpp
	plugin-silo.h
	silo-shapefile.cpp
	silo-sphereVol.cpp
	#silo-ddOutput.cpp
	silo-voronoi.cpp
	#CurveMesh3d.cpp
	#CurveMesh3d.h
	#MaterialList.cpp
	#MaterialList.h
	#QuadMesh3d.cpp
	#QuadMesh3d.h
	#RectilinearMesh3d.cpp
	#RectilinearMesh3d.h
	#remake.h
	#WritePoints.cpp
	#WritePoints.h
	)
addlib(rtmath_plugin_silo SHARED)

target_link_libraries(rtmath_plugin_silo ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat silo rtmath_silo_cpp)
addplugin(rtmath_plugin_silo export rtmath)

