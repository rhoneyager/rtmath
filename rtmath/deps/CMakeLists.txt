option ( DEPS_TMATRIX
	"Build the code with tmatrix support (needs FORTRAN compiler)" ON)
if (DEPS_TMATRIX)
	add_subdirectory (tmatrix)
endif()
