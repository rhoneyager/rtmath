include (common)

add_executable (rtmath-expand-shorthand expand.cpp)
set_target_properties( rtmath-expand-shorthand PROPERTIES FOLDER "Apps/Core")
target_link_libraries(rtmath-expand-shorthand ${rtmath_core-libs} rtmath_core)
INSTALL(TARGETS rtmath-expand-shorthand RUNTIME DESTINATION bin COMPONENT Applications)

if(BUILD_TESTING)
	add_test (paramSet rtmath-expand-shorthand 0)
	set_tests_properties (paramSet
		PROPERTIES PASS_REGULAR_EXPRESSION "Expansion of"
		)
endif()

