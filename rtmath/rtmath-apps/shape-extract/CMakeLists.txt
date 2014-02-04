include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-shape-extract shape-extract.cpp)
set_target_properties( rtmath-shape-extract PROPERTIES FOLDER "Apps/Shapes")
target_link_libraries(rtmath-shape-extract ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-shape-extract RUNTIME DESTINATION bin COMPONENT Applications)


if (BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-shape-extract rtmath_plugin_silo)
endif()
if (BUILD_PLUGIN_BOV)
	add_dependencies ( rtmath-shape-extract rtmath_plugin_bov)
endif()
if (BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-shape-extract rtmath_plugin_hdf5)
endif()



