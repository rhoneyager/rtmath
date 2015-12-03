cmake_minimum_required (VERSION 2.8)
project(scatdb_ryan CXX C Fortran)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

include(common)
include(SVNinfo)
include(addBoostUniform)
include(addBaseProject)
include(addInstallDirs)
include(addModuleFiles)
set(MAJOR 0)
set(MINOR 1)
set(REVISION 0)
set(SCATDB_RYAN_VERSION
	${MAJOR}.${MINOR}.${REVISION}.${SVNREVISION} )
addBaseProject()
addInstallDirs(scatdb_ryan ${SCATDB_RYAN_VERSION})
addModuleFiles(scatdb_ryan ${SCATDB_RYAN_VERSION})

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

add_subdirectory (../spline "${CMAKE_CURRENT_BINARY_DIR}/spline")

configure_file (
        "${CMAKE_CURRENT_SOURCE_DIR}/module-scatdb-ryan.in"
        "${CMAKE_CURRENT_BINARY_DIR}/module-scatdb-ryan"
        )

add_library (scatdb_ryan SHARED
	scatdb_ryan/defs.h
	scatdb_ryan/defs.hpp
	scatdb_ryan/lowess.h
	scatdb_ryan/scatdb_ryan.h
	scatdb_ryan/scatdb_ryan.hpp
	scatdb_ryan/utility.hpp
	src/scatdb_ryan.cpp
	src/scatdb_ryan_c.cpp
	src/filters.cpp
	src/lowess.cpp
	src/io.cpp
	src/utility.cpp
	${CMAKE_CURRENT_BINARY_DIR}/module-scatdb-ryan
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
	set(libs Ryan_Debug spline)
else()
	set (libs Ryan_Debug spline ${Boost_FILESYSTEM_LIBRARY} ${Boost_IOSTREAMS_LIBRARY}
		${Boost_PROGRAM_OPTIONS_LIBRARY}
		${Boost_SYSTEM_LIBRARY})
endif()
target_link_libraries(scatdb_ryan ${libs})

add_executable (scatdb_ryan_test
	src/test.cpp
	)
set_target_properties( scatdb_ryan_test PROPERTIES FOLDER "Related")
target_link_libraries(scatdb_ryan_test ${libs} scatdb_ryan)

add_executable (scatdb_ryan_test_c
	src/test-c.c
	)
set_target_properties( scatdb_ryan_test_c PROPERTIES FOLDER "Related")
target_link_libraries(scatdb_ryan_test_c ${libs} scatdb_ryan)

add_executable (scatdb_ryan_test_f
	src/test-f.f90
	)
set_target_properties( scatdb_ryan_test_f PROPERTIES FOLDER "Related")
set_target_properties( scatdb_ryan_test_f PROPERTIES LINKER_LANGUAGE Fortran)
target_link_libraries(scatdb_ryan_test_f ${libs} scatdb_ryan)

add_executable (scatdb_ryan_envelopes src/envelopes.cpp)
set_target_properties(scatdb_ryan_envelopes PROPERTIES FOLDER "Related")
target_link_libraries(scatdb_ryan_envelopes ${libs} scatdb_ryan)

INSTALL(TARGETS scatdb_ryan scatdb_ryan_test scatdb_ryan_test_c
	scatdb_ryan_test_f scatdb_ryan_envelopes
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	COMPONENT Plugins)

# Install modulefiles on linux / unix / mac
if (NOT WIN32)
	if (INSTALL_MODULES)
		INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/module-scatdb-ryan
			DESTINATION ${ENV_MOD_DIR_PREFIX}
			COMPONENT Data
			RENAME ${MODULES_FILENAME}
			)
	endif()
endif()

