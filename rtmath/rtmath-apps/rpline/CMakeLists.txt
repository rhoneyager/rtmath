include (common)
include (commonRtmathApps)

add_executable (rtmath-rpline rpline.cpp)
addapp( rtmath-rpline Core)
target_link_libraries(rtmath-rpline ${rtmath_core-libs} rtmath_core )


