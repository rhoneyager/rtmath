include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-radar-relations radar-relations.cpp)
set_target_properties( rtmath-radar-relations PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-radar-relations ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-radar-relations RUNTIME DESTINATION bin COMPONENT Applications)

if (BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-radar-relations rtmath_plugin_hdf5)
endif()

if (BUILD_PLUGIN_TSV)
	add_dependencies ( rtmath-radar-relations rtmath_plugin_tsv)
endif()



