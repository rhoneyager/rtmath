include (common)
include (commonRtmathApps)

add_executable (rtmath-oneellip-table
	main.cpp
	)
target_link_libraries( rtmath-oneellip-table 
	rtmath_core 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	)
addapp(rtmath-oneellip-table Analysis)

if (BUILD_PLUGINS AND BUILD_PLUGIN_BHMIE)
	add_dependencies ( rtmath-oneellip-table rtmath_plugin_bhmie)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_MIE)
	add_dependencies ( rtmath-oneellip-table rtmath_plugin_mie)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_NONDDA_MISC)
	add_dependencies ( rtmath-oneellip-table rtmath_plugin_nondda_misc)
endif()
if (BUILD_PLUGINS AND BUILD_PLUGIN_TMATRIX)
	add_dependencies ( rtmath-oneellip-table rtmath_plugin_tmatrix)
endif()

