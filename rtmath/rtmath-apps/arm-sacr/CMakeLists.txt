include (common)
include (commonRtmathApps)

add_executable (rtmath-arm-sacr main.cpp)
target_link_libraries(rtmath-arm-sacr ${rtmath_data-libs} rtmath_core rtmath_data)
addapp(rtmath-arm-sacr Data)

#if (BUILD_PLUGIN_IMAGEMAGICK)
#	add_dependencies ( rtmath-arm-sacr rtmath_plugin_ImageMagick)
#endif()

# Add a database plugin dependency, too...
if (BUILD_PLUGIN_NETCDF)
	add_dependencies ( rtmath-arm-sacr rtmath_plugin_netcdf)
endif()
if (BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-arm-sacr rtmath_plugin_tsv)
endif()


