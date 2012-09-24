include (common)
set(rtmath-mie_LIBRARIES "")
set(rtmath-mie_INCLUDE_DIRS "")

add_executable (rtmath-mie main.cpp)
add_package_dependency(rtmath-mie DEPENDS_ON ROOT)

addapp(rtmath-mie)

TARGET_LINK_LIBRARIES( rtmath-mie tmatrix-cpp tmatrix-fortran)
