include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-shape-hash shape-hash.cpp)
set_target_properties( rtmath-shape-hash PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-shape-hash ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-shape-hash RUNTIME DESTINATION bin COMPONENT Applications)



