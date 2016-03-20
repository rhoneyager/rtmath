include (common)
include (commonRtmathApps)

add_executable (rtmath-lowess lowess.cpp lowess.h main.cpp)
set_target_properties( rtmath-lowess PROPERTIES FOLDER "Apps/Core")
target_link_libraries(rtmath-lowess ${rtmath_core-libs} rtmath_core )
addapp(rtmath-lowess Data)


