include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-shape-extract shape-extract.cpp)
set_target_properties( rtmath-shape-extract PROPERTIES FOLDER "Apps/Shapes")
target_link_libraries(rtmath-shape-extract ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-shape-extract RUNTIME DESTINATION bin COMPONENT Applications)



