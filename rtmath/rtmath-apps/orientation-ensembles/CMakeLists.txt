include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-orientation-ensembles orientation-ensembles.cpp)
set_target_properties( rtmath-orientation-ensembles PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-orientation-ensembles ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-orientation-ensembles RUNTIME DESTINATION bin COMPONENT Applications)



