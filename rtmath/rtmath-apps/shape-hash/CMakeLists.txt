include (common)
include (commonRtmathApps)

add_executable (rtmath-shape-hash shape-hash.cpp)
addapp( rtmath-shape-hash Analysis)
target_link_libraries(rtmath-shape-hash ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)



