include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-ddpar main.cpp)
target_link_libraries(rtmath-ddscat-ddpar ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-ddpar Analysis)


