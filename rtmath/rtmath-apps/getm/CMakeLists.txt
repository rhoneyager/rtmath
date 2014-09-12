include (common)
include (commonRtmathApps)

add_executable (rtmath-getm getm.cpp)
addapp( rtmath-getm Analysis)
target_link_libraries(rtmath-getm ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)


