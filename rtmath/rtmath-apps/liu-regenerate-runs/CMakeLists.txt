include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-liu-regenerate-runs main.cpp)
set_target_properties( rtmath-liu-regenerate-runs PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-liu-regenerate-runs ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-liu-regenerate-runs RUNTIME DESTINATION bin COMPONENT Applications)



