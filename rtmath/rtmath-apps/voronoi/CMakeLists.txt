include (common)
include (commonRtmathApps)

add_executable (rtmath-voronoi voronoi-main.cpp)
addapp( rtmath-voronoi Shapes)
target_link_libraries(rtmath-voronoi ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)

if (BUILD_PLUGINS AND BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-voronoi rtmath_plugin_silo)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-voronoi rtmath_plugin_hdf5)
endif()




