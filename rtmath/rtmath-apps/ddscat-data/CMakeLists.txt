include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-data data.cpp)
set_target_properties( rtmath-ddscat-data PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-ddscat-data ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-data RUNTIME DESTINATION bin COMPONENT Applications)



