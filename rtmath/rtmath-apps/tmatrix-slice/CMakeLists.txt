#include (common)

include_directories (
        ${CMAKE_BINARY_DIR}
)

add_executable (rtmath-tmatrix-slice
	main.cpp
	)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(tmatrix REQUIRED)
target_link_libraries( rtmath-tmatrix-slice 
	rtmath_core 
	rtmath_mie 
	rtmath_ddscat 
	${rtmath_core-libs}
	${rtmath_mie-libs}
	${rtmath_ddscat-libs}
	tmatrix-cpp 
	tmatrix-fortran 
	)

set_target_properties( rtmath-tmatrix-slice PROPERTIES FOLDER "Apps")
include_directories(${CMAKE_CURRENT_BINARY_DIR})
INSTALL(TARGETS rtmath-tmatrix-slice RUNTIME DESTINATION bin COMPONENT Applications)

