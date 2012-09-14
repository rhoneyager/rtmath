include (common)

add_executable (rtmath-mie main.cpp)
addapp(rtmath-mie)

TARGET_LINK_LIBRARIES( rtmath-mie tmatrix-cpp tmatrix-fortran)
