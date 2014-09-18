include (common)
include (commonRtmathApps)

add_executable (rtmath-ddscat-regenerate-run-db regenerate-run-db.cpp)
target_link_libraries(rtmath-ddscat-regenerate-run-db ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
addapp(rtmath-ddscat-regenerate-run-db Analysis)

