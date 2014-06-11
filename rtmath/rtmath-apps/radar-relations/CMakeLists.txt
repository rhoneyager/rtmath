include (common)
include (commonRtmathApps)

add_executable (rtmath-radar-relations radar-relations.cpp)
addapp( rtmath-radar-relations Analysis)
target_link_libraries(rtmath-radar-relations ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)

if (BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-radar-relations rtmath_plugin_hdf5)
endif()

if (BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-radar-relations rtmath_plugin_tsv)
endif()



