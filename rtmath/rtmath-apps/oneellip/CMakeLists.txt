include (common)
include (commonRtmathApps)

add_executable (rtmath-oneellip
	main.cpp
	)
target_link_libraries( rtmath-oneellip 
	rtmath_core 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	)
addapp(rtmath-oneellip Analysis)

if (BUILD_PLUGIN_TMATRIX)
	add_dependencies ( rtmath-oneellip rtmath_plugin_tmatrix)
endif()

if (BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-oneellip rtmath_plugin_tsv)
endif()

