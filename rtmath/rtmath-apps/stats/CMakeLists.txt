include (common)
include (commonRtmathApps)

add_executable (rtmath-stats stats-main.cpp)
addapp( rtmath-stats Shapes)
target_link_libraries(rtmath-stats ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)

if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-stats rtmath_plugin_hdf5)
endif()




