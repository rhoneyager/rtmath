include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-generate-run generate-run.cpp)
set_target_properties( rtmath-ddscat-generate-run PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-ddscat-generate-run ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-generate-run RUNTIME DESTINATION bin COMPONENT Applications)


