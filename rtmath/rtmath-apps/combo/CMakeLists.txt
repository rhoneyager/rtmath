include (common)
include (commonRtmathApps)

add_executable (rtmath-combo combo2.cpp)
set_target_properties( rtmath-combo PROPERTIES FOLDER "Apps/Core")
target_link_libraries(rtmath-combo ${rtmath_core-libs} rtmath_core )
addapp(rtmath-combo Data)


