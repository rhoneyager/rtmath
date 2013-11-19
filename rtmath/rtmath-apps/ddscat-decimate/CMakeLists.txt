include (common)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#find_package(PCL REQUIRED COMPONENTS common io octree segmentation surface)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})

add_executable (rtmath-ddscat-decimate main.cpp)
set_target_properties( rtmath-ddscat-decimate PROPERTIES FOLDER "Apps/Shapes"")
target_link_libraries(rtmath-ddscat-decimate ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat
	#	${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_OCTREE_LIBRARIES} 
	#${PCL_SEGMENTATION_LIBRARIES} ${PCL_SURFACE_LIBRARIES}
	)
INSTALL(TARGETS rtmath-ddscat-decimate RUNTIME DESTINATION bin COMPONENT Applications)


