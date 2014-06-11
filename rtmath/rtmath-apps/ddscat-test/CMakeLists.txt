include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-test ddscat-test.cpp)
addapp( rtmath-ddscat-test Analysis)
target_link_libraries(rtmath-ddscat-test ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat
	)


