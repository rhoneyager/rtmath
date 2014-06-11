include (common)
include (commonRtmathApps)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})
find_package(Voro REQUIRED)
include_directories(BEFORE ${VORO_INCLUDE_DIRS})

add_executable (rtmath-orientation-output-table orientation-output-table.cpp)
addapp( rtmath-orientation-output-table Analysis)
target_link_libraries(rtmath-orientation-output-table ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
target_link_libraries(rtmath-orientation-output-table ${VORO_LIBRARIES})




