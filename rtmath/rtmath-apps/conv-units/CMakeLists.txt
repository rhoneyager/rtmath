include (common)
include (commonRtmathApps)

add_executable (rtmath-conv-units conv.cpp)
target_link_libraries(rtmath-conv-units ${rtmath_core-libs} rtmath_core)
addapp(rtmath-conv-units Core)


