include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
#message("Include ${ImageMagick_LIBRARIES}")
include_directories(${ImageMagick_INCLUDE_DIRS})
add_definitions(-DNOAUTOLINK_MAGICK)
add_library (rtmath_plugin_ImageMagick SHARED 
	plugin-ImageMagick.cpp
	plugin-ImageMagick.h
	image-io.cpp
	)
addlib(rtmath_plugin_ImageMagick SHARED)

target_link_libraries(rtmath_plugin_ImageMagick 
	${rtmath_ddscat-libs} 
	rtmath_core 
	rtmath_ddscat 
	rtmath_images
	${rtmath_images-libs}
	${ImageMagick_LIBRARIES})
addplugin(rtmath_plugin_ImageMagick Shapes rtmath)


