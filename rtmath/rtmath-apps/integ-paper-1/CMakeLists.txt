include (common)
include (commonRtmathApps)

add_executable (rtmath-integ-paper-1
	main.cpp
	)
target_link_libraries( rtmath-integ-paper-1 
	rtmath_core 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_ddscat-libs}
	)
addapp(rtmath-integ-paper-1 Analysis)

