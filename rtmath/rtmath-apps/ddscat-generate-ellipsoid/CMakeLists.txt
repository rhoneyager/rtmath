include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-generate-ellipsoid main.cpp)
target_link_libraries(rtmath-ddscat-generate-ellipsoid ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-generate-ellipsoid Shapes)

