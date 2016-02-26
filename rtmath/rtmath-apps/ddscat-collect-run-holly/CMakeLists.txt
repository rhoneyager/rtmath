include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-collect-run-holly collect-run-holly.cpp)
target_link_libraries(rtmath-ddscat-collect-run-holly ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-collect-run-holly Analysis)

