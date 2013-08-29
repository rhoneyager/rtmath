include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-output-extract ddscat-output-extract.cpp)
set_target_properties( rtmath-ddscat-output-extract PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-ddscat-output-extract ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-output-extract RUNTIME DESTINATION bin COMPONENT Applications)



