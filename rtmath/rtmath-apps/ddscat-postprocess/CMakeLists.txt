include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-postprocess postprocess.cpp)
target_link_libraries(rtmath-ddscat-postprocess ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-postprocess Analysis)


