include (common)
include (commonRtmathApps)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-arm-coverage coverage.cpp)
target_link_libraries(rtmath-arm-coverage ${rtmath_data-libs} rtmath_core rtmath_data)
addapp(rtmath-arm-coverage Data)

#if (BUILD_PLUGIN_IMAGEMAGICK)
#	add_dependencies ( rtmath-arm-coverage rtmath_plugin_ImageMagick)
#endif()

if (BUILD_PLUGIN_PSQL)
	add_dependencies ( rtmath-arm-coverage rtmath_plugin_psql)
endif()
if (BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-arm-coverage rtmath_plugin_tsv)
endif()


