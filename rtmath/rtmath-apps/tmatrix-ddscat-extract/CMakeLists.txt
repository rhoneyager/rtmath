#include (common)

include_directories (
	${CMAKE_BINARY_DIR}
)

add_executable (rtmath-tmatrix-ddscat-extract
	main.cpp 
	)

find_package(tmatrixFortran REQUIRED)
find_package(tmatrixCpp REQUIRED)
target_link_libraries( rtmath-tmatrix-ddscat-extract 
	rtmath_core 
	rtmath_mie 
	rtmath_ddscat
	${rtmath_core-libs}
	${rtmath_mie-libs}
	${rtmath_ddscat-libs}
	tmatrix-cpp
	tmatrix-fortran
	)

set_target_properties( rtmath-tmatrix-ddscat-extract PROPERTIES FOLDER "Apps/Tmatrix")
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${TMATRIX_CPP_INCLUDE_DIRS})
INSTALL(TARGETS rtmath-tmatrix-ddscat-extract RUNTIME DESTINATION bin COMPONENT Applications)

