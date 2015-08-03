include (common)
include (commonRtmathApps)

add_executable (rtmath-density
	main.cpp
	)
target_link_libraries( rtmath-density 
	rtmath_core 
	rtmath_ddscat
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	)
addapp(rtmath-density Analysis)


