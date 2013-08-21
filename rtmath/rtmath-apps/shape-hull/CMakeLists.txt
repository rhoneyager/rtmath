include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})

add_executable (rtmath-shape-hull shape-hull.cpp)
set_target_properties( rtmath-shape-hull PROPERTIES FOLDER "Apps/Voronoi")
target_link_libraries(rtmath-shape-hull ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-shape-hull RUNTIME DESTINATION bin COMPONENT Applications)



