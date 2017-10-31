include (common)
include (commonRtmathApps)

add_executable (rtmath-image-basic main.cpp)
addapp( rtmath-image-basic Images)
target_link_libraries(rtmath-image-basic ${rtmath_images-libs} rtmath_core rtmath_images)

if (BUILD_PLUGINS AND BUILD_PLUGIN_IMAGEMAGICK)
	add_dependencies ( rtmath-image-basic rtmath_plugin_ImageMagick)
endif()

if (BUILD_PLUGINS AND BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-image-basic rtmath_plugin_tsv)
endif()
