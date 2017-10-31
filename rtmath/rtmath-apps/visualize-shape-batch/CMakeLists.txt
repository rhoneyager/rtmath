include (common)
include (commonRtmathApps)

add_executable (rtmath-visualize-shape-batch visualize-shape-batch.cpp)
target_link_libraries(rtmath-visualize-shape-batch ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-visualize-shape-batch Visualization)

#	add_dependencies ( rtmath-visualize-shape-batch rtmath-voronoi )

if (BUILD_PLUGINS AND BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-visualize-shape-batch rtmath_plugin_silo)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-visualize-shape-batch rtmath_plugin_hdf5)
endif()
