include (common)
include (commonRtmathApps)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})

add_executable (rtmath-shape-perturb-interior shape-perturb-interior.cpp)
addapp( rtmath-shape-perturb-interior Voronoi)
target_link_libraries(rtmath-shape-perturb-interior ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)

if (BUILD_PLUGINS AND BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-shape-perturb-interior rtmath_plugin_silo)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_BOV)
	add_dependencies ( rtmath-shape-perturb-interior rtmath_plugin_bov)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-shape-perturb-interior rtmath_plugin_hdf5)
endif()



