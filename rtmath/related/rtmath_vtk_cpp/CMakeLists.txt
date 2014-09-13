cmake_minimum_required (VERSION 2.8)
project(rtmath_vtk_cpp CXX C)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

include(common)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})
include(${VTK_USE_FILE})


include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../../rtmath")

add_library (rtmath_vtk_cpp STATIC
	WriteVTK.cpp
	WriteVTK.h
	)
if(WIN32 AND NOT CYGWIN)
else()
	add_definitions(-fPIC)
endif()
set_target_properties( rtmath_vtk_cpp PROPERTIES FOLDER "Related")
target_link_libraries(rtmath_vtk_cpp  ${libs} ${VTK_LIBRARIES})

INSTALL(TARGETS rtmath_vtk_cpp
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	COMPONENT Plugins)

