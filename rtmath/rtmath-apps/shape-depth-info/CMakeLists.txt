include (common)
include (commonRtmathApps)

add_executable (rtmath-shape-depth-info shape-depth-info.cpp)
addapp( rtmath-shape-depth-info Voronoi)
target_link_libraries(rtmath-shape-depth-info ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)

if (BUILD_PLUGINS AND BUILD_PLUGIN_VTK)
	add_dependencies ( rtmath-shape-depth-info rtmath_plugin_vtk)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_VORO)
	add_dependencies ( rtmath-shape-depth-info rtmath_plugin_voro)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-shape-depth-info rtmath_plugin_hdf5)
endif()



