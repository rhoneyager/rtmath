include (common)
include (commonRtmathApps)

add_executable (rtmath-version rtmath-version.cpp)
target_link_libraries(rtmath-version ${rtmath_core-libs} rtmath_core)
addapp(rtmath-version Core)

if(BUILD_TESTING)
	add_test (rtmath-core-lib-link rtmath-version)
	set_tests_properties (rtmath-core-lib-link
		PROPERTIES PASS_REGULAR_EXPRESSION "rtmath library"
		)
endif()

