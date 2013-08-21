include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})
find_package(Voro REQUIRED)
include_directories(BEFORE ${VORO_INCLUDE_DIRS})

add_executable (rtmath-orientation-weights-ddscat orientation-weights-ddscat.cpp)
set_target_properties( rtmath-orientation-weights-ddscat PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-orientation-weights-ddscat ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
target_link_libraries(rtmath-orientation-weights-ddscat ${VORO_LIBRARIES})

INSTALL(TARGETS rtmath-orientation-weights-ddscat RUNTIME DESTINATION bin COMPONENT Applications)



