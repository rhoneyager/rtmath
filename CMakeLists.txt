# CMake file for Ryan's serialization library
# Designed to build a shared library (though static is also okay)

# Will handle easy project imports

cmake_minimum_required (VERSION 2.8)
project(Ryan_Debug C CXX)

# Add path for custom modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/"
)

# Set the svn variables
include (SVNinfo)

set(MAJOR 0)
set(MINOR 6)
set(REVISION 2)

set(RYAN_DEBUG_VERSION
	${MAJOR}.${MINOR}.${REVISION}.${SVNREVISION})

set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for headers")
set(INSTALL_DOC_DIR share/doc/Ryan_Debug-${RYAN_DEBUG_VERSION} CACHE PATH "Installation directory for documentation")
if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR ${INSTALL_LIB_DIR}/CMake/Ryan_Debug)
endif()

if(WIN32 AND NOT CYGWIN)
	set (modinstall OFF)
else()
	set (modinstall ON)
endif()
option ( INSTALL_MODULES
	"Install the environment-modules package file. Does nothing on Windows." ${modinstall})
SET( ENV_MOD_DIR_PREFIX
	/etc/modulefiles
	CACHE STRING
	"Environment-modules package file install directory"
)
set(MODULES_FILENAME
	Ryan_Debug
	CACHE STRING
	"Name of module file"
)

set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
	"Installation directory for CMake files")

find_package(Doxygen)
option (BUILD_DOCUMENTATION
	"Build the documentation for this library" OFF)

if(BUILD_DOCUMENTATION)

	option (BUILD_DOCUMENTATION_IN_ALL
		"Build documentation automatically with 'make all'. Also used for 'make install' and 'make package'" OFF)

	if (NOT DOXYGEN_FOUND)
		message(SEND_ERROR "Documentation build requested but Doxygen is not found.")
	endif()


	configure_file(Doxyfile.in
		"${PROJECT_BINARY_DIR}/Doxyfile" @ONLY)

	if (BUILD_DOCUMENTATION_IN_ALL)
		set (ALL_FLAG ALL)
	else()
		set (ALL_FLAG "")
	endif()

	# This builds the html docs
	add_custom_target(doc-html ${ALL_FLAG}
		${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
		COMMENT "Generating API html documentation with Doxygen" VERBATIM 
		)
	# This builds the latex docs
	add_custom_target(doc-latex ${ALL_FLAG}
		latex refman.tex 
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/docs/latex
		COMMENT "Generating API pdf documentation with Doxygen" VERBATIM
		)

	add_custom_target(docs ${ALL_FLAG} DEPENDS doc-html doc-latex)
endif()

# Configure the subversion.h file
configure_file(
	"debug_subversion.h.template"
	"${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h" @ONLY)


# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

configure_file(src/lib.rc.in
	"${PROJECT_BINARY_DIR}/lib.rc" @ONLY)

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE DOC)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()
 
configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/environment/ryan-debug"
	"${PROJECT_BINARY_DIR}/module-ryan-debug"
	)

find_package(Boost COMPONENTS filesystem program_options system REQUIRED)
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
	Ryan_Debug/debug.h
	Ryan_Debug/info.h
	${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h
	)

#set(srcs-test src/test.cpp)

if(WIN32)
	set (srcs ${srcs} "${PROJECT_BINARY_DIR}/lib.rc" )
endif()

source_group("Header Files" FILES ${hdrs})
source_group("Source Files" FILES ${srcs})


add_library(Ryan_Debug SHARED ${srcs} ${hdrs} 
	"${PROJECT_BINARY_DIR}/module-ryan-debug"
	)
target_link_libraries(Ryan_Debug ${liblist})
set_target_properties(Ryan_Debug PROPERTIES FOLDER "Libs")

#add_executable(Ryan_Debug-test ${srcs-test})
#target_link_libraries(Ryan_Debug-test ${liblist} Ryan_Debug)
#set_target_properties(Ryan_Debug-test PROPERTIES FOLDER "Apps")

export(TARGETS Ryan_Debug
	FILE "${PROJECT_BINARY_DIR}/Ryan_DebugTargets.cmake")
export(PACKAGE Ryan_Debug)

file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
	"${INSTALL_INCLUDE_DIR}")

# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(Ryan-DebugConfig.cmake.in
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfig.cmake" @ONLY)
# ... and for the install tree
set(CONF_INCLUDE_DIRS "\${RYAN_DEBUG_CMAKE_DIR}/${REL_INCLUDE_DIR}")

#message("${CONF_INCLUDE_DIRS}")

configure_file(Ryan-DebugConfig.cmake.in
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan_DebugConfig.cmake" @ONLY)
# ... for both
configure_file(Ryan-DebugConfigVersion.cmake.in
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfigVersion.cmake" @ONLY)
 
if(WIN32 AND NOT CYGWIN) # NSIS bug
INSTALL(TARGETS Ryan_Debug
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Libraries
	)
endif()

INSTALL(TARGETS Ryan_Debug
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	LIBRARY DESTINATION ${INSTALL_LIB_DIR}
	ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
	COMPONENT Libraries
	)

INSTALL(DIRECTORY Ryan-Debug/
#	EXPORT Ryan_DebugTargets
	DESTINATION ${INSTALL_INCLUDE_DIR}/Ryan-Debug
	COMPONENT Headers
	FILES_MATCHING PATTERN "*.h"
	)

IF(INSTALL_MODULES AND NOT WIN32)
	INSTALL(
		FILES ${PROJECT_BINARY_DIR}/module-ryan-debug
		DESTINATION ${ENV_MOD_DIR_PREFIX}
		COMPONENT Environment
		RENAME ${MODULES_FILENAME}
	)
ENDIF()

add_subdirectory(apps)

#INSTALL(TARGETS Ryan_Debug-test
#	EXPORT Ryan_DebugTargets
#	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
#	COMPONENT Applications
#	)

if (BUILD_DOCUMENTATION_IN_ALL)
	# Provides html and pdf
	install(CODE "execute_process(COMMAND ${CMAKE_BUILD_TOOL} docs)")
	# html
	install(DIRECTORY ${CMAKE_BINARY_DIR}/docs/html/ DESTINATION ${INSTALL_DOC_DIR}/html)
	# pdf
	install(DIRECTORY ${CMAKE_BINARY_DIR}/docs/latex/ DESTINATION ${INSTALL_DOC_DIR}/latex)
	# man
	install(DIRECTORY ${CMAKE_BINARY_DIR}/docs/man/ DESTINATION ${INSTALL_DOC_DIR}/man)
endif()


INSTALL(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan_DebugConfig.cmake"
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfigVersion.cmake"
	DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT Libraries)

INSTALL(EXPORT Ryan_DebugTargets DESTINATION
	"${INSTALL_CMAKE_DIR}" COMPONENT Libraries)

set(CPACK_PACKAGE_NAME "Ryan_Debug")
set(CPACK_PACKAGE_VENDOR "Ryan Honeyager")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Ryan_Debug - Ryan's debugging library")
set(CPACK_PACKAGE_VERSION "${RYAN_DEBUG_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH ${REVISION})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Ryan_Debug")
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

#set(CPACK_SET_DESTDIR 1)
set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
#message("${CPACK_PACKAGING_INSTALL_PREFIX}")
#set(CPACK_RPM_USER_BINARY_SPECFILE "${PROJECT_BINARY_DIR}/ryan-debug.spec")
#set(CPACK_RPM_USER_BINARY_SPECFILE "${PROJECT_SOURCE_DIR}/ryan-debug.spec.in")

# Fixing RPM EXPORT issue
#set(CPACK_IGNORE_FILES ".*EXPORT$" ".*Ryan_DebugTargets$")
#set(CPACK_SOURCE_IGNORE_FILES ${CPACK_IGNORE_FILES})

set(CPACK_RPM_PACKAGE_REQUIRES
        "boost-devel >= 1.45, boost-filesystem, gcc-c++ >= 4.6"
        )

set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "libboost-dev (>= 1.46.1), libboost-filesystem-dev (>= 1.46.1), g++ (>= 4:4.6.1)"
        )

# This must always be last!
include(CPack)


