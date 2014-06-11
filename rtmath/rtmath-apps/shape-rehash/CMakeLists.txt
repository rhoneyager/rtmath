include (common)
include (commonRtmathApps)

add_executable (rtmath-shape-rehash shape-rehash.cpp)
addapp( rtmath-shape-rehash Analysis)
target_link_libraries(rtmath-shape-rehash ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)



