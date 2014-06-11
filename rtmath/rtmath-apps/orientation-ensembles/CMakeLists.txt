include (common)
include (commonRtmathApps)

add_executable (rtmath-orientation-ensembles orientation-ensembles.cpp)
addapp( rtmath-orientation-ensembles Analysis)
target_link_libraries(rtmath-orientation-ensembles ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)



