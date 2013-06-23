include (common)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-test ddscat-test.cpp)
set_target_properties( rtmath-ddscat-test PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-ddscat-test ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat
	)
INSTALL(TARGETS rtmath-ddscat-test RUNTIME DESTINATION bin COMPONENT Applications)


