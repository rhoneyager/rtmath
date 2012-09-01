include (common)

add_executable (rtmath-expand-shorthand expand.cpp)
addapp(rtmath-expand-shorthand)

if(BUILD_TESTING)
	add_test (paramSet rtmath-expand-shorthand 0)
	set_tests_properties (rtmath-lib-link
		PROPERTIES PASS_REGULAR_EXPRESSION "Expansion of"
		)
endif()

