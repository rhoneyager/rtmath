include (common)
include (commonRtmathApps)

add_executable (rtmath-vf
	main.cpp
	)
target_link_libraries( rtmath-vf 
	rtmath_core 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	)
addapp(rtmath-vf Analysis)

