include (common)
include (commonRtmathApps)
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/"
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Flann REQUIRED)
include_directories(${FLANN_INCLUDE_DIRS})

add_library (rtmath_plugin_flann SHARED 
	plugin-flann.cpp
	plugin-flann.h
	)
addlib(rtmath_plugin_flann SHARED)

target_link_libraries(rtmath_plugin_flann ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat ${FLANN_LIBRARIES})
addplugin(rtmath_plugin_flann Shapes rtmath)

