include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#find_package(ZLIB REQUIRED)
#include_directories(${ZLIB_INCLUDE_DIRS})
#find_package(SZIP REQUIRED)

add_library (rtmath_plugin_tsv SHARED 
	plugin-tsv.cpp
	plugin-tsv.h
	tsv-shapestats.cpp
	tsv-shapestats-rotdata.cpp
	tsv-shapestats-voro.cpp
	)
addlib(rtmath_plugin_tsv SHARED)

set_target_properties( rtmath_plugin_tsv PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_tsv ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat ${ROOT_LIBRARIES})
INSTALL(TARGETS rtmath_plugin_tsv 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

