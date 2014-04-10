include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(ROOT REQUIRED)
include_directories(${ROOT_INCLUDE_DIR})
find_package(HDF5 COMPONENTS C)
if (HDF5_FOUND)
include_directories(${HDF5_INCLUDE_DIRS})
add_definitions(${HDF5_DEFINITIONS})
endif()
#find_package(ZLIB REQUIRED)
#include_directories(${ZLIB_INCLUDE_DIRS})
#find_package(SZIP REQUIRED)

add_library (rtmath_plugin_root SHARED 
	plugin-root.cpp
	plugin-root.h
	root-sacr.cpp
	# root-shapestats.cpp
	)
addlib(rtmath_plugin_root SHARED)

set_target_properties( rtmath_plugin_root PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_root 
	${rtmath_ddscat-libs} rtmath_core rtmath_ddscat 
	${rtmath_data-libs} rtmath_data
	${ROOT_LIBRARIES})
INSTALL(TARGETS rtmath_plugin_root 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

