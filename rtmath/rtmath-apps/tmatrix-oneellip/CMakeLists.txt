#include (common)

include_directories (
        ${CMAKE_BINARY_DIR}
)

add_executable (rtmath-tmatrix-oneellip
	main.cpp
	)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(tmatrix REQUIRED)
target_link_libraries( rtmath-tmatrix-oneellip 
	rtmath_core 
	rtmath_mie 
	${rtmath_core-libs}
	${rtmath_mie-libs}
	tmatrix-cpp
	tmatrix-fortran
	)

set_target_properties( rtmath-tmatrix-oneellip PROPERTIES FOLDER "Apps")
include_directories(${CMAKE_CURRENT_BINARY_DIR})
INSTALL(TARGETS rtmath-tmatrix-oneellip RUNTIME DESTINATION bin COMPONENT Applications)

