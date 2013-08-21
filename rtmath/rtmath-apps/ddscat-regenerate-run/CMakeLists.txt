include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-regenerate-run regenerate-run.cpp)
set_target_properties( rtmath-ddscat-regenerate-run PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-ddscat-regenerate-run ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-regenerate-run RUNTIME DESTINATION bin COMPONENT Applications)


