include (common)
include (commonRtmathApps)

add_executable (rtmath-expand-shorthand expand.cpp)
addapp( rtmath-expand-shorthand Core)
target_link_libraries(rtmath-expand-shorthand ${rtmath_core-libs} rtmath_core)

if(BUILD_TESTING)
	add_test (paramSet rtmath-expand-shorthand 0)
	set_tests_properties (paramSet
		PROPERTIES PASS_REGULAR_EXPRESSION "Expansion of"
		)
endif()

