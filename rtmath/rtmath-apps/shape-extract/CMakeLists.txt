include (common)
include (commonRtmathApps)

add_executable (rtmath-shape-extract shape-extract.cpp)
addapp( rtmath-shape-extract Shapes)
target_link_libraries(rtmath-shape-extract ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)


if (BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-shape-extract rtmath_plugin_silo)
endif()
if (BUILD_PLUGIN_BOV)
	add_dependencies ( rtmath-shape-extract rtmath_plugin_bov)
endif()
if (BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-shape-extract rtmath_plugin_hdf5)
endif()



