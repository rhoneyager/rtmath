include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-shape-basicdata main.cpp)

set_target_properties( rtmath-shape-basicdata PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-shape-basicdata ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-shape-basicdata RUNTIME DESTINATION bin COMPONENT Applications)

