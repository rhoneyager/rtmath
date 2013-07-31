include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(BEFORE ${VTK_INCLUDE_DIRS})

add_executable (rtmath-vtktest vtktest.cpp)
set_target_properties( rtmath-vtktest PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-vtktest Ryan_Debug)
#target_link_libraries(rtmath-vtktest ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
target_link_libraries(rtmath-vtktest ${VTK_LIBRARIES})


INSTALL(TARGETS rtmath-vtktest RUNTIME DESTINATION bin COMPONENT Applications)



