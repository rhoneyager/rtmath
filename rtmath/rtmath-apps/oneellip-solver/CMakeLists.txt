include (common)
include (commonRtmathApps)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable (rtmath-oneellip-solver
	main.cpp
	)
target_link_libraries( rtmath-oneellip-solver 
	rtmath_core 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	)
addapp(rtmath-oneellip-solver Analysis)

if (BUILD_PLUGIN_BHMIE)
	add_dependencies ( rtmath-oneellip-solver rtmath_plugin_bhmie)
endif()
if (BUILD_PLUGIN_MIE)
	add_dependencies ( rtmath-oneellip-solver rtmath_plugin_mie)
endif()
if (BUILD_PLUGIN_NONDDA_MISC)
	add_dependencies ( rtmath-oneellip-solver rtmath_plugin_nondda_misc)
endif()
if (BUILD_PLUGIN_TMATRIX)
	add_dependencies ( rtmath-oneellip-solver rtmath_plugin_tmatrix)
endif()

