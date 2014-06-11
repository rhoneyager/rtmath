include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-regenerate-run regenerate-run.cpp)
target_link_libraries(rtmath-ddscat-regenerate-run ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-regenerate-run Analysis)

