cmake_minimum_required (VERSION 2.8)
project(kuo_flake_extract CXX C)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

include(common)
include(addBoostUniform)

find_package(Ryan_Debug REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${RYAN_DEBUG_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set (boost_uniform_libs iostreams program_options filesystem system)
addBoostUniform(${boost_uniform_libs})
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

find_package(HDF5 COMPONENTS C CXX REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS} ${HDF5_INCLUDE_DIR_CPP})
add_definitions(${HDF5_DEFINITIONS})

if(DEFINED HDF5_C_LIBRARIES)
	set (h5libs ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
else()
	set (h5libs hdf5 hdf5_cpp)
endif()
#FIND_PACKAGE(SZIP)
#	message("${SZIP_INCLUDE_DIRS}")
#	message("${SZIP_LIBRARIES}")
#IF (DEFINED SZIP_LIBRARIES)
#	include_directories(${SZIP_INCLUDE_DIRS})
#		SET (h5libs ${h5libs} szip)
#ENDIF()
if(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
if(WIN32 AND NOT CYGWIN)
else()
	add_definitions(-fPIC)
endif()
if(WIN32 AND NOT CYGWIN)
	set(libs Ryan_Debug)
else()
	set (libs Ryan_Debug ${Boost_FILESYSTEM_LIBRARY} ${Boost_IOSTREAMS_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY}
		${Boost_SYSTEM_LIBRARY})
endif()
add_subdirectory (../rtmath_hdf5_cpp "${CMAKE_CURRENT_BINARY_DIR}/rtmath_hdf5_cpp")

#include_directories("${CMAKE_CURRENT_BINARY_DIR}/../../rtmath")


add_executable (kuo_flake_extract 
	read.cpp
	main.cpp
	read.h
	)
set_target_properties(kuo_flake_extract PROPERTIES FOLDER "Related")
target_link_libraries(kuo_flake_extract ${libs}
	${h5libs}
	rtmath_hdf5_cpp
	)

INSTALL(TARGETS kuo_flake_extract
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	COMPONENT Plugins)

