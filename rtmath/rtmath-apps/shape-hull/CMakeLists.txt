include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-shape-hull shape-hull.cpp)
set_target_properties( rtmath-shape-hull PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-shape-hull ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-shape-hull RUNTIME DESTINATION bin COMPONENT Applications)



