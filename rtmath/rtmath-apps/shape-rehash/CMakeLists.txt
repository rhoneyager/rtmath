include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-shape-rehash shape-rehash.cpp)
set_target_properties( rtmath-shape-rehash PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-shape-rehash ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-shape-rehash RUNTIME DESTINATION bin COMPONENT Applications)



