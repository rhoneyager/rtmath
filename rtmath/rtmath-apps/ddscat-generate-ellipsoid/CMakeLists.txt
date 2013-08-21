include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-generate-ellipsoid main.cpp)
set_target_properties( rtmath-ddscat-generate-ellipsoid PROPERTIES FOLDER "Apps/Shapes")
target_link_libraries(rtmath-ddscat-generate-ellipsoid ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-generate-ellipsoid RUNTIME DESTINATION bin COMPONENT Applications)


