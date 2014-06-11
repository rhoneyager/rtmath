include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-output-extract ddscat-output-extract.cpp)
target_link_libraries(rtmath-ddscat-output-extract ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-output-extract Analysis)


