include (common)

add_executable (rtmath-version rtmath-version.cpp)
set_target_properties( rtmath-version PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-version ${rtmath_core-libs} rtmath_core)
INSTALL(TARGETS rtmath-version RUNTIME DESTINATION bin COMPONENT Applications)

if(BUILD_TESTING)
	add_test (rtmath-core-lib-link rtmath-version)
	set_tests_properties (rtmath-core-lib-link
		PROPERTIES PASS_REGULAR_EXPRESSION "rtmath library"
		)
endif()

