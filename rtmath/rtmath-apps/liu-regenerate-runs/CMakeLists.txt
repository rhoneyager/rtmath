include (common)
include (commonRtmathApps)

add_executable (rtmath-liu-regenerate-runs main.cpp)
addapp( rtmath-liu-regenerate-runs Analysis)
target_link_libraries(rtmath-liu-regenerate-runs ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)



