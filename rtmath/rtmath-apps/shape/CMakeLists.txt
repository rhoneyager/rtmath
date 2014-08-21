include (common)
include (commonRtmathApps)

add_executable (rtmath-shape shape-main.cpp)
addapp( rtmath-shape Shapes)
target_link_libraries(rtmath-shape ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)

if (BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-shape rtmath_plugin_silo)
endif()
if (BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-shape rtmath_plugin_hdf5)
endif()
if (BUILD_PLUGIN_PSQL)
	add_dependencies ( rtmath-shape rtmath_plugin_psql)
endif()



