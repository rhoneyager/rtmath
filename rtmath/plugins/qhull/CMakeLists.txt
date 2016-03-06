include (common)
include (commonRtmathApps)
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/"
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Qhull REQUIRED)
include_directories(${QHULL_INCLUDE_DIRS})

add_library (rtmath_plugin_qhull SHARED 
	plugin-qhull.cpp
	plugin-qhull.h
	qhull-hulls.cpp
	)
addlib(rtmath_plugin_qhull SHARED)

target_link_libraries(rtmath_plugin_qhull
	${rtmath_ddscat-libs} rtmath_core rtmath_ddscat
	${QHULL_LIBRARIES} )
addplugin(rtmath_plugin_qhull Shapes rtmath)

