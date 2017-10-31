include (common)
include (commonRtmathApps)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})

add_executable (rtmath-shape-hull shape-hull.cpp)
addapp( rtmath-shape-hull Voronoi)
target_link_libraries(rtmath-shape-hull ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)

if (BUILD_PLUGINS AND BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-shape-hull rtmath_plugin_silo)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_BOV)
	add_dependencies ( rtmath-shape-hull rtmath_plugin_bov)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-shape-hull rtmath_plugin_hdf5)
endif()



