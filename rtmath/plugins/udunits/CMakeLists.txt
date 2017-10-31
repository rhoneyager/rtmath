include (common)
include (commonRtmathApps)
#find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
#find_package(Ryan_Debug REQUIRED)
find_package(Udunits2 REQUIRED)
include_directories(${UDUNITS2_INCLUDE_DIR})

add_library (rtmath_plugin_udunits SHARED 
	plugin-udunits.cpp
	plugin-udunits.h
	)
addlib(rtmath_plugin_udunits SHARED)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


target_link_libraries(rtmath_plugin_udunits ${rtmath_core-libs} rtmath_core
	${UDUNITS2_LIBRARIES})
addplugin(rtmath_plugin_udunits core rtmath)

