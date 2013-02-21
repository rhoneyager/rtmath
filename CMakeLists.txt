# CMake file for Ryan's serialization library
# Designed to build a shared library (though static is also okay)

# Will handle easy project imports

cmake_minimum_required (VERSION 2.8)
project(ryan-debug C CXX)

option (BUILD_SHARED
	"Build a dynamic link library" ON)
option (BUILD_STATIC
	"Build a static library" ON)

set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INC_DIR include CACHE PATH "Installation directory for headers")

set(liblist ${Boost_LIBRARIES})

find_package(Boost COMPONENTS filesystem iostreams serialization system REQUIRED)
set( srcs
	src/os_functions.cpp
	)
set (hdrs
	ryan-debug/debug.h
	)

source_group("Header Files" FILES ${hdrs})
source_group("Source Files" FILES ${srcs})

include_directories(${Boost_INCLUDE_DIRS})

if(BUILD_SHARED)
	add_library(ryan-debug SHARED ${srcs} ${hdrs} )
	target_link_libraries(ryan-debug ${liblist})
	set_target_properties(ryan-debug PROPERTIES FOLDER "Libs")
	INSTALL(TARGETS ryan-debug
		RUNTIME DESTINATION ${INSTALL_BIN_DIR}
		LIBRARY DESTINATION ${INSTALL_LIB_DIR}
		ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
		COMPONENT Libraries
		)
endif()
if(BUILD_STATIC)
	add_library(ryan-debug_static STATIC ${srcs} ${hdrs} )
	target_link_libraries(ryan-debug_static ${liblist})
	set_target_properties(ryan-debug_static PROPERTIES FOLDER "Libs")
	INSTALL(TARGETS ryan-debug_static
		RUNTIME DESTINATION ${INSTALL_BIN_DIR}
		LIBRARY DESTINATION ${INSTALL_LIB_DIR}
		ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
		COMPONENT Libraries
		)
endif()

INSTALL(DIRECTORY ryan-debug/
	DESTINATION ${INSTALL_INC_DIR}/ryan-debug
	COMPONENT Headers
	FILES_MATCHING PATTERN "*.h"
	)
