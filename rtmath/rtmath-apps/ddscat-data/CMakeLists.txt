include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-data data.cpp)
target_link_libraries(rtmath-ddscat-data ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-data Analysis)



