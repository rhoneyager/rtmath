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
include(SVNinfo)
include(addBaseProject)
include(addlib)
include(addAssembly)
include(addconfiginfo)
include(addInstallDirs)
include(addModuleFiles)
include(addBoostUniform)
include(adddocs)
include(generateAssemblyGuid)

set(MAJOR 0)
set(MINOR 7)
set(REVISION 3)

set(RYAN_DEBUG_VERSION
	${MAJOR}.${MINOR}.${REVISION}.${SVNREVISION})

math(EXPR PREVSVNREVISION "${SVNREVISION} - 1")

addBaseProject()
addInstallDirs(Ryan_Debug ${RYAN_DEBUG_VERSION})
addModuleFiles(Ryan_Debug ${RYAN_DEBUG_VERSION})
addAssembly(Ryan_Debug)

# Configure the subversion.h file
configure_file(
	"debug_subversion.h.template"
	"${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h" @ONLY)


# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

configure_file(src/lib.rc.in
	"${PROJECT_BINARY_DIR}/lib.rc" @ONLY)
configure_file(src/versioninfo.h.in
	"${PROJECT_BINARY_DIR}/versioninfo.h" @ONLY)

addBoostUniform(filesystem program_options system)

include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

set( srcs
	src/modules.cpp
	src/os_functions.cpp
	)
set (hdrs
	Ryan_Debug/defs.h
	Ryan_Debug/debug.h
	Ryan_Debug/info.h
	Ryan_Debug/modules.h
	src/internal.h
	${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h
	)

#set(srcs-test src/test.cpp)

if(WIN32 AND NOT CYGWIN)
	set (srcs ${srcs} "${PROJECT_BINARY_DIR}/lib.rc" )
endif()

source_group("Header Files" FILES ${hdrs})
source_group("Source Files" FILES ${srcs})

if(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

add_library(Ryan_Debug SHARED ${srcs} ${hdrs} 
	"${PROJECT_BINARY_DIR}/module-ryan-debug"
	)
target_link_libraries(Ryan_Debug ${liblist})
if(WIN32 AND NOT CYGWIN)
ELSE()
	target_link_libraries(Ryan_Debug dl)
ENDIF()
addlib(Ryan_Debug SHARED)
if (NOT DEFINED MAKE_ASSEMBLY)
	signing(Ryan_Debug) # Will be part of the assembly code
endif()

if (MAKE_ASSEMBLY)
	foreach(p Debug RelWithDebInfo MinSizeRel Release)
		set(ASSEMBLY_NAME ${ASSEMBLY_NAME_${p}})
		set(ASSEMBLY_LIB_NAME Ryan_Debug_${p}${configappend}.dll)
		#set(ASSEMBLY_MANIFEST_DIR bin//)
		generateAssemblyGuid(Ryan_Debug ${p} policy sxspolicyguid)
		generateAssemblyGuid(Ryan_Debug ${p} public sxspublicguid)
		configure_file(
			"Ryan.Debug.DebugAssembly.manifest.template"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/${ASSEMBLY_NAME_${p}}.pre.manifest" @ONLY)
		configure_file(
			"${CMAKE_CURRENT_SOURCE_DIR}/sxs.template.wxs"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/sxs_${ASSEMBLY_NAME_${p}}.wxs" @ONLY)	
	endforeach()
	implementAssembly(Ryan_Debug Ryan_Debug )
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.template.wxs"
		"${CMAKE_CURRENT_BINARY_DIR}/config${configappend}.wxs" @ONLY)
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Ryan_Debug.template.wxs"
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.wxs" @ONLY)
	INSTALL(FILES
		"${CMAKE_CURRENT_BINARY_DIR}/config${configappend}.wxs"
		DESTINATION "${INSTALL_INCLUDE_DIR}/../installer" COMPONENT Scripts)
	INSTALL(FILES
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.wxs"
		"${CMAKE_CURRENT_SOURCE_DIR}/upgrade.bat"
		"${CMAKE_CURRENT_SOURCE_DIR}/collectdirs.bat"
		"${CMAKE_CURRENT_SOURCE_DIR}/deploy.bat"
		"${CMAKE_CURRENT_SOURCE_DIR}/license.rtf"
		DESTINATION "${INSTALL_INCLUDE_DIR}/.." COMPONENT Scripts)
endif()

add_subdirectory(apps)

#add_executable(Ryan_Debug-test ${srcs-test})
#target_link_libraries(Ryan_Debug-test ${liblist} Ryan_Debug)
#set_target_properties(Ryan_Debug-test PROPERTIES FOLDER "Apps")

export(TARGETS Ryan_Debug
	FILE "${PROJECT_BINARY_DIR}/Ryan_DebugTargets.cmake")
export(PACKAGE Ryan_Debug)

 
configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/environment/ryan-debug"
	"${PROJECT_BINARY_DIR}/module-ryan-debug"
	)


#file(RELATIVE_PATH REL_INCLUDE_DIR "${ABS_INSTALL_CMAKE_DIR}"
#	"${INSTALL_INCLUDE_DIR}")

# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(Ryan-DebugConfig.cmake.in
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfig.cmake" @ONLY)
# ... and for the install tree
set(CONF_INCLUDE_DIRS "\${RYAN_DEBUG_CMAKE_DIR}/${REL_INCLUDE_DIR}") # Before WiX
#set(CONF_INCLUDE_DIRS "")

#message("${CONF_INCLUDE_DIRS}")

configure_file(Ryan-DebugConfig.cmake.in
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan_DebugConfig.cmake" @ONLY)
# ... for both
configure_file(Ryan-DebugConfigVersion.cmake.in
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfigVersion.cmake" @ONLY)

#message("${INSTALL_CMAKE_DIR}/${REL_ASSEMBLY_DIR}")

#foreach(p Debug RelWithDebInfo MinSizeRel Release)
	if(WIN32 AND NOT CYGWIN) # NSIS bug
	INSTALL(TARGETS Ryan_Debug
		RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
		COMPONENT Libraries
		#CONFIGURATIONS ${p}
		)
	endif()

	INSTALL(TARGETS Ryan_Debug
		EXPORT Ryan_DebugTargets
		RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
		LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
		ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}/lib${configappend}
		COMPONENT Libraries
		#CONFIGURATIONS ${p}
		)

	INSTALL(EXPORT Ryan_DebugTargets DESTINATION
		${INSTALL_CMAKE_DIR} COMPONENT Libraries 
		#CONFIGURATIONS ${p}
		)

#endforeach()



INSTALL(DIRECTORY Ryan_Debug/
	DESTINATION ${INSTALL_INCLUDE_DIR}/Ryan_Debug
	COMPONENT Headers
	FILES_MATCHING PATTERN "*.h"
	)

IF(INSTALL_MODULES)
	INSTALL(
		FILES ${PROJECT_BINARY_DIR}/module-ryan-debug
		DESTINATION ${ENV_MOD_DIR_PREFIX}
		COMPONENT Environment
		RENAME ${MODULES_FILENAME}
	)
ENDIF()

adddocs()

INSTALL(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan_DebugConfig.cmake"
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfigVersion.cmake"
	DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT Libraries)


