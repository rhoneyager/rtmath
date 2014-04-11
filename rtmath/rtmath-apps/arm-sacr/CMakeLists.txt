include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-arm-sacr main.cpp)
set_target_properties( rtmath-arm-sacr PROPERTIES FOLDER "Apps/Data")
target_link_libraries(rtmath-arm-sacr ${rtmath_data-libs} rtmath_core rtmath_data)
INSTALL(TARGETS rtmath-arm-sacr RUNTIME DESTINATION bin COMPONENT Applications)

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


