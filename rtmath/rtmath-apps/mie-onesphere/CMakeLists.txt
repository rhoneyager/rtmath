#include (common)

include_directories (
        ${CMAKE_BINARY_DIR}
)

add_executable (rtmath-mie-onesphere
	main.cpp
	)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries( rtmath-mie-onesphere 
	rtmath_core 
	rtmath_mie 
	${rtmath_core-libs}
	${rtmath_mie-libs}
	)

set_target_properties( rtmath-mie-onesphere PROPERTIES FOLDER "Apps")
include_directories(${CMAKE_CURRENT_BINARY_DIR})
INSTALL(TARGETS rtmath-mie-onesphere RUNTIME DESTINATION bin COMPONENT Applications)

