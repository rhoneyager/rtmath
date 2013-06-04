include (common)

include_directories (
	${CMAKE_BINARY_DIR}
)

add_executable (rtmath-tmatrix-ddscat-extract
	main.cpp 
	)

#add_package_dependency(rtmath-tmatrix-ddscat-extract DEPENDS_ON ROOT)

addapp(rtmath-tmatrix-ddscat-extract)
TARGET_LINK_LIBRARIES( rtmath-tmatrix-ddscat-extract tmatrix-cpp tmatrix-fortran)

