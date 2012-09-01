include (common)

add_executable (rtmath-version rtmath-version.cpp)
addapp(rtmath-version)

if(BUILD_TESTING)
	add_test (rtmath-lib-link rtmath-version)
	set_tests_properties (rtmath-lib-link
		PROPERTIES PASS_REGULAR_EXPRESSION "rtmath library"
		)
endif()

