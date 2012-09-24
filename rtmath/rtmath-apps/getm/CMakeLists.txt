include (common)
set(rtmath-getm_LIBRARIES "")
set(rtmath-getm_INCLUDE_DIRS "")
set(rtmath-genmtab_LIBRARIES "")
set(rtmath-genmtab_INCLUDE_DIRS "")

add_executable (rtmath-getm getm.cpp)
add_package_dependency(rtmath-getm DEPENDS_ON ROOT)
addapp(rtmath-getm)
add_executable (rtmath-genmtab getm.cpp)
add_package_dependency(rtmath-genmtab DEPENDS_ON ROOT)
addapp(rtmath-genmtab)

