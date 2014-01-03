include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(HDF5 COMPONENTS C CXX REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS})
add_definitions(${HDF5_DEFINITIONS})

add_library (rtmath-plugin-hdf5 SHARED plugin-hdf5.cpp)
addlib(rtmath-plugin-hdf5 SHARED)

set_target_properties( rtmath-plugin-hdf5 PROPERTIES FOLDER "Plugins/ddscat")
target_link_libraries(rtmath-plugin-hdf5 
	${rtmath_ddscat-libs} rtmath_core rtmath_ddscat
	${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
INSTALL(TARGETS rtmath-plugin-hdf5 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

