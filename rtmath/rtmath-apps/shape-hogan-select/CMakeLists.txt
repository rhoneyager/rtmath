include (common)
include (commonRtmathApps)

add_executable (rtmath-shape-hogan-select shape-hogan-select.cpp)
addapp( rtmath-shape-hogan-select Shapes)
target_link_libraries(rtmath-shape-hogan-select ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)


