include (common)
include (commonRtmathApps)

add_executable (rtmath-shape-match-parent shape-match-parent.cpp)
target_link_libraries(rtmath-shape-match-parent ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-shape-match-parent Shapes)

