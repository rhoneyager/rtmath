include (common)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (rtmath-ddscat-output ddscat-output.cpp)
set_target_properties( rtmath-ddscat-output PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-ddscat-output ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-ddscat-output RUNTIME DESTINATION bin COMPONENT Applications)

if (BUILD_PLUGIN_SILO)
	add_dependencies ( rtmath-ddscat-output rtmath_plugin_silo)
endif()
if (BUILD_PLUGIN_HDF5)
	add_dependencies ( rtmath-ddscat-output rtmath_plugin_hdf5)
endif()


