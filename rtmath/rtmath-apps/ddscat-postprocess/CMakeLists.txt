include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-postprocess postprocess.cpp)
set_target_properties( rtmath-ddscat-postprocess PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-ddscat-postprocess ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-postprocess RUNTIME DESTINATION bin COMPONENT Applications)



