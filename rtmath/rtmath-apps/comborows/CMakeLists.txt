include (common)
include (commonRtmathApps)

add_executable (rtmath-comborows comborows.cpp)
target_link_libraries(rtmath-comborows ${rtmath_core-libs} rtmath_core )
addapp(rtmath-comborows Core)


