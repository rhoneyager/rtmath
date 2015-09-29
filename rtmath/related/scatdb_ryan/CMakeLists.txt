cmake_minimum_required (VERSION 2.8)
project(scatdb_ryan CXX C)

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
#include_directories(BEFORE
#	"${CMAKE_SOURCE_DIR}/related/linterp/rncarpio-linterp-ca556a0/src")

add_library (scatdb_ryan SHARED
	scatdb_ryan/defs.h
	scatdb_ryan/defs.hpp
	scatdb_ryan/scatdb_ryan.h
	scatdb_ryan/scatdb_ryan.hpp
	scatdb_ryan/utility.hpp
	src/scatdb_ryan.cpp
	src/io.cpp
	src/utility.cpp
	)
if(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()
if(WIN32 AND NOT CYGWIN)
else()
	add_definitions(-fPIC)
endif()
set_target_properties( scatdb_ryan PROPERTIES FOLDER "Related")
if(WIN32 AND NOT CYGWIN)
	set(libs Ryan_Debug)
else()
	set (libs Ryan_Debug ${Boost_FILESYSTEM_LIBRARY} ${Boost_IOSTREAMS_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY}
		${Boost_SYSTEM_LIBRARY})
endif()
target_link_libraries(scatdb_ryan ${libs})

add_executable (scatdb_ryan_test
	src/test.cpp
	)
set_target_properties( scatdb_ryan_test PROPERTIES FOLDER "Related")
target_link_libraries(scatdb_ryan_test ${libs} scatdb_ryan)

INSTALL(TARGETS scatdb_ryan
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	COMPONENT Plugins)

