include (common)
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

set_target_properties( rtmath_plugin_ImageMagick PROPERTIES FOLDER "Plugins/Shapes")
target_link_libraries(rtmath_plugin_ImageMagick 
	${rtmath_ddscat-libs} 
	rtmath_core 
	rtmath_ddscat 
	rtmath_images
	${rtmath_images-libs}
	${ImageMagick_LIBRARIES})
INSTALL(TARGETS rtmath_plugin_ImageMagick 
	LIBRARY DESTINATION ${INSTALL_PLUGIN_DIR} 
	ARCHIVE DESTINATION ${INSTALL_PLUGIN_DIR} 
	COMPONENT Plugins)

