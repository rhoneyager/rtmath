include (common)
include (commonRtmathApps)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-arm-info main.cpp)
target_link_libraries(rtmath-arm-info ${rtmath_data-libs} rtmath_core rtmath_data)
addapp(rtmath-arm-info Data)

if (BUILD_PLUGINS AND BUILD_PLUGIN_PSQL)
	add_dependencies ( rtmath-arm-info rtmath_plugin_psql)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_NETCDF)
	add_dependencies ( rtmath-arm-info rtmath_plugin_netcdf)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_ROOT)
	add_dependencies ( rtmath-arm-info rtmath_plugin_root)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-arm-info rtmath_plugin_tsv)
endif()


