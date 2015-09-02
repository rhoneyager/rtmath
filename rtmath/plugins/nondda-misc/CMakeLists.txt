include (common)
include (commonRtmathApps)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library (rtmath_plugin_nondda_misc SHARED 
	nondda-rayleigh.cpp
	nondda-rg.cpp
	nondda-rg-iso.cpp
	nondda-hrg.cpp
	nondda-hrg-iso.cpp
	plugin-nondda-misc.cpp
	plugin-nondda-misc.h
	)
addlib(rtmath_plugin_nondda_misc SHARED)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


target_link_libraries(rtmath_plugin_nondda_misc ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat )
addplugin(rtmath_plugin_nondda_misc non_dda rtmath)

