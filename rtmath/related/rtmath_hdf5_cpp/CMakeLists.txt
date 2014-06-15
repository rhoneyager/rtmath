cmake_minimum_required (VERSION 2.8)
project(rtmath_hdf5_cpp CXX C)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

include(common)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(HDF5 COMPONENTS C CXX REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR_CPP})
add_definitions(${HDF5_DEFINITIONS})

if(DEFINED HDF5_C_LIBRARIES)
	set (h5libs ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
else()
	set (h5libs hdf5 hdf5_cpp)
endif()
FIND_PACKAGE(SZIP)
#	message("${SZIP_INCLUDE_DIRS}")
#	message("${SZIP_LIBRARIES}")
IF (DEFINED SZIP_LIBRARIES)
	include_directories(${SZIP_INCLUDE_DIRS})
		SET (h5libs ${h5libs} szip)
ENDIF()

add_library (rtmath_hdf5_cpp STATIC
	export-hdf5.cpp
	export-hdf5.h
	)
add_definitions(-fPIC)
set_target_properties( rtmath_hdf5_cpp PROPERTIES FOLDER "Related")
target_link_libraries(rtmath_hdf5_cpp ${h5libs} ${libs})

INSTALL(TARGETS rtmath_hdf5_cpp
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	COMPONENT Plugins)

