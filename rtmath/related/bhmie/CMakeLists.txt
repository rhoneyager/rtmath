cmake_minimum_required (VERSION 2.8)
project(rtmath_bhmie CXX C)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

include(common)

add_library (rtmath_bhmie STATIC
	bhmie.c
	bhmie.h
	complex.c
	complex.h
	nrutil.c
	nrutil.h
	)
if(WIN32 AND NOT CYGWIN)
else()
	add_definitions(-fPIC)
endif()
set_target_properties( rtmath_bhmie PROPERTIES FOLDER "Related")

INSTALL(TARGETS rtmath_bhmie
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}/plugins
	COMPONENT Plugins)

