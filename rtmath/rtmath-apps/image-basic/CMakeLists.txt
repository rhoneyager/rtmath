include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-image-basic main.cpp)
set_target_properties( rtmath-image-basic PROPERTIES FOLDER "Apps/Images")
target_link_libraries(rtmath-image-basic ${rtmath_images-libs} rtmath_core rtmath_images)
INSTALL(TARGETS rtmath-image-basic RUNTIME DESTINATION bin COMPONENT Applications)

if (BUILD_PLUGIN_IMAGEMAGICK)
	add_dependencies ( rtmath-image-basic rtmath_plugin_ImageMagick)
endif()

if (BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-image-basic rtmath_plugin_tsv)
endif()