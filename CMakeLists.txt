# CMake file for Ryan's serialization library
# Designed to build a shared library (though static is also okay)

# Will handle easy project imports

cmake_minimum_required (VERSION 2.8)
project(Ryan-Debug C CXX)

set(MAJOR 0)
set(MINOR 5)
set(REVISION 1)
set(RYAN-DEBUG_VERSION
	${MAJOR}.${MINOR}.${REVISION})

set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for headers")
if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
	set (INSTEXT ON)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/CMake/Ryan-Debug)
	set (INSTEXT OFF)
endif()
option (INSTALL_EXTERNALS
	"Install external library dll files?" ${INSTEXT})
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
	"Installation directory for CMake files")
 
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()
 


find_package(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${BOOST_INCLUDEDIR})
if (WIN32 AND NOT CYGWIN) # Boost auto-link
	link_directories(${BOOST_LIBRARYDIR})
else()
	set(liblist ${Boost_LIBRARIES})
endif()
#message("${liblist}")

set( srcs
	src/os_functions.cpp
	)
set (hdrs
	Ryan-Debug/debug.h
	Ryan-Debug/info.h
	)

set(srcs-test src/test.cpp)

if(WIN32)
	set (srcs ${srcs} src/lib.rc)
endif()

source_group("Header Files" FILES ${hdrs})
source_group("Source Files" FILES ${srcs})


add_library(Ryan-Debug SHARED ${srcs} ${hdrs} )
target_link_libraries(Ryan-Debug ${liblist})
set_target_properties(Ryan-Debug PROPERTIES FOLDER "Libs")

add_executable(Ryan-Debug-test ${srcs-test})
target_link_libraries(Ryan-Debug-test ${liblist} Ryan-Debug)
set_target_properties(Ryan-Debug-test PROPERTIES FOLDER "Apps")

export(TARGETS Ryan-Debug
	FILE "${PROJECT_BINARY_DIR}/Ryan-DebugTargets.cmake")
export(PACKAGE Ryan-Debug)

file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
	"${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")

# For the local tree
configure_file(Ryan-DebugConfig.cmake.in
	"${PROJECT_BINARY_DIR}/Ryan-DebugConfig.cmake" @ONLY)
# ... and for the install tree
set(CONF_INCLUDE_DIRS "\${RYAN-DEBUG_CMAKE_DIR}/${REL_INCLUDE_DIR}")

#message("${CONF_INCLUDE_DIRS}")

configure_file(Ryan-DebugConfig.cmake.in
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan-DebugConfig.cmake" @ONLY)
# ... for both
configure_file(Ryan-DebugConfigVersion.cmake.in
	"${PROJECT_BINARY_DIR}/Ryan-DebugConfigVersion.cmake" @ONLY)
 

if(WIN32 AND NOT CYGWIN) # NSIS bug
INSTALL(TARGETS Ryan-Debug
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Libraries
	)
endif()

INSTALL(TARGETS Ryan-Debug
	EXPORT Ryan-DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	LIBRARY DESTINATION ${INSTALL_LIB_DIR}
	ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
	COMPONENT Libraries
	)

INSTALL(DIRECTORY Ryan-Debug/
	EXPORT Ryan-DebugTargets
	DESTINATION ${INSTALL_INCLUDE_DIR}/Ryan-Debug
	COMPONENT Headers
	FILES_MATCHING PATTERN "*.h"
	)

INSTALL(TARGETS Ryan-Debug-test
	EXPORT Ryan-DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Applications
	)

INSTALL(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan-DebugConfig.cmake"
	"${PROJECT_BINARY_DIR}/Ryan-DebugConfigVersion.cmake"
	DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT Libraries)

INSTALL(EXPORT Ryan-DebugTargets DESTINATION
	"${INSTALL_CMAKE_DIR}" COMPONENT Libraries)

# Also install / pack the relevant boost libraries into the install tree

# Do extention swapping
if(WIN32)
	set(dllext ".dll")
	set(libext "[.]lib")
else()
	set(dllext ".so")
	set(libext "[.]a")
endif()

STRING ( REGEX REPLACE ${libext} ${dllext} FILESYSTEM_D_DLL ${Boost_FILESYSTEM_LIBRARY_DEBUG} )
STRING ( REGEX REPLACE ${libext} ${dllext} FILESYSTEM_R_DLL ${Boost_FILESYSTEM_LIBRARY_RELEASE} )
STRING ( REGEX REPLACE ${libext} ${dllext} SYSTEM_D_DLL ${Boost_SYSTEM_LIBRARY_DEBUG} )
STRING ( REGEX REPLACE ${libext} ${dllext} SYSTEM_R_DLL ${Boost_SYSTEM_LIBRARY_RELEASE} )

if (INSTALL_EXTERNALS)
	INSTALL(FILES 
		${FILESYSTEM_D_DLL}
		${SYSTEM_D_DLL}
		DESTINATION ${INSTALL_BIN_DIR}
		COMPONENT Libraries
		CONFIGURATIONS Debug
		)

	INSTALL(FILES 
		${FILESYSTEM_R_DLL}
		${SYSTEM_R_DLL}
		DESTINATION ${INSTALL_BIN_DIR}
		COMPONENT Libraries
		CONFIGURATIONS Release;RelWithDebInfo;MinSizeRel
		)
endif()

set(CPACK_PACKAGE_NAME "Ryan-Debug")
set(CPACK_PACKAGE_VENDOR "Ryan Honeyager")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Ryan-Debug - Ryan's debugging library")
set(CPACK_PACKAGE_VERSION "${RYAN-DEBUG_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH ${REVISION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Ryan-Debug")
set(CPACK_PACKAGE_CONTACT "Ryan Honeyager (rhoneyager@fsu.edu)")

set (CPACK_COMPONENTS_ALL
        Libraries
        Headers
	Applications
)

set(CPACK_COMPONENT_APPLICATIONS_DESCRIPTION
	"The testing apps")
set(CPACK_COMPONENT_LIBRARIES_DESCRIPTION
        "The compiled libraries")
set(CPACK_COMPONENT_HEADERS_DESCRIPTION
        "Headers for code development")

set(CPACK_COMPONENT_HEADERS_DEPENDS Libraries)
set(CPACK_COMPONENT_APPLICATIONS_DEPENDS Libraries)
set(CPACK_COMPONENT_LIBRARIES_REQUIRED 1)

set(CPACK_RPM_PACKAGE_REQUIRES
        "boost-devel >= 1.45, boost-filesystem, gcc-c++ >= 4.6"
        )

set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "libboost-dev (>= 1.46.1), libboost-filesystem-dev (>= 1.46.1), g++ (>= 4:4.6.1)"
        )

# This must always be last!
include(CPack)


