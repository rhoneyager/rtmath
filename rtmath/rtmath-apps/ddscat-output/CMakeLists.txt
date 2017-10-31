include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-output ddscat-output.cpp)
target_link_libraries(rtmath-ddscat-output ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-output Analysis)

if (BUILD_PLUGINS AND BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-ddscat-output rtmath_plugin_silo)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-ddscat-output rtmath_plugin_hdf5)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-ddscat-output rtmath_plugin_tsv)
endif()

