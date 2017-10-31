include (common)
include (commonRtmathApps)

add_execusolver-basic (rtmath-oneellip-solver-basic
	main.cpp
	)
target_link_libraries( rtmath-oneellip-solver-basic 
	rtmath_core 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	)
addapp(rtmath-oneellip-solver-basic Analysis)

if (BUILD_PLUGINS AND BUILD_PLUGIN_TMATRIX)
	add_dependencies ( rtmath-oneellip-solver-basic rtmath_plugin_tmatrix)
endif()

