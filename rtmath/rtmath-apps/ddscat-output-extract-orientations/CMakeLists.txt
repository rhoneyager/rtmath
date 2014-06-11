include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-output-extract-orientations ddscat-output-extract-orientations.cpp)
target_link_libraries(rtmath-ddscat-output-extract-orientations ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-output-extract-orientations Analysis)


