include (common)
include (commonRtmathApps)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})
find_package(Voro REQUIRED)
include_directories(BEFORE ${VORO_INCLUDE_DIRS})

add_executable (rtmath-visualize-orientation-weights visualize-orientation-weights.cpp)
addapp( rtmath-visualize-orientation-weights Visualization)
target_link_libraries(rtmath-visualize-orientation-weights Ryan_Debug)
target_link_libraries(rtmath-visualize-orientation-weights ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
target_link_libraries(rtmath-visualize-orientation-weights ${VTK_LIBRARIES})




