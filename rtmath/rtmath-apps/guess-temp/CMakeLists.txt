include (common)
include (commonRtmathApps)

add_executable (rtmath-guess-temp guess.cpp)
target_link_libraries(rtmath-guess-temp ${rtmath_core-libs} rtmath_core)
addapp(rtmath-guess-temp Core)


