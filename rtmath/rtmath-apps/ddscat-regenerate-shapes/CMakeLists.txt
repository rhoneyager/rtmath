include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-regenerate-shapes main.cpp)
set_target_properties( rtmath-ddscat-regenerate-shapes PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-ddscat-regenerate-shapes ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-regenerate-shapes RUNTIME DESTINATION bin COMPONENT Applications)


