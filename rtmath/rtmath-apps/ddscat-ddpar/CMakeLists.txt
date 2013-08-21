include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-ddpar main.cpp)
set_target_properties( rtmath-ddscat-ddpar PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-ddscat-ddpar ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-ddpar RUNTIME DESTINATION bin COMPONENT Applications)


