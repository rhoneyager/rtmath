include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-output-ingest ddscat-output-ingest.cpp)
target_link_libraries(rtmath-ddscat-output-ingest ${rtmath_core-libs} rtmath_core )
addapp(rtmath-ddscat-output-ingest Analysis)

#if (BUILD_PLUGIN_HDF5)
#	add_dependencies ( rtmath-ddscat-output-ingest rtmath_plugin_hdf5)
#endif()


