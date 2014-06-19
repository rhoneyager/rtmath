include (common)
include (commonRtmathApps)

add_executable (rtmath-visualize-shape-batch-fast visualize-shape-batch-fast.cpp)
target_link_libraries(rtmath-visualize-shape-batch-fast ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-visualize-shape-batch-fast Visualization)

#	add_dependencies ( rtmath-visualize-shape-batch-fast rtmath-voronoi )

if (BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-visualize-shape-batch-fast rtmath_plugin_silo)
endif()
if (BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-visualize-shape-batch-fast rtmath_plugin_hdf5)
endif()
