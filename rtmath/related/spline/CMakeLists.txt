cmake_minimum_required (VERSION 2.8)
project(spline CXX C Fortran)

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
set(MAJOR 1)
set(MINOR 0)
set(REVISION 0)
set(SPLINE_VERSION
	${MAJOR}.${MINOR}.${REVISION}.${SVNREVISION} )
addBaseProject()
addInstallDirs(spline ${SPLINE_VERSION})
addModuleFiles(spline ${SPLINE_VERSION})

add_library (spline STATIC
	spline.cpp
	spline.hpp
	)
if(WIN32 AND NOT CYGWIN)
else()
	add_definitions(-fPIC)
endif()
set_target_properties( spline PROPERTIES FOLDER "Related")

add_executable (spline_test
	spline_prb.cpp
	)
set_target_properties( spline_test PROPERTIES FOLDER "Related")
target_link_libraries(spline_test spline)


INSTALL(TARGETS spline spline_test 
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
	COMPONENT Plugins)

