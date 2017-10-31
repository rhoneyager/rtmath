# CMake file for Ryan's Debugging library
# Designed to build a shared library (though static is also okay)

# Will handle easy project imports

set(RDMAJOR 0)
set(RDMINOR 11)
set(RDREVISION 1)

set(RYAN_DEBUG_VERSION
	${RDMAJOR}.${RDMINOR}.${RDREVISION})

configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake-settings.h.in"
	"${CMAKE_BINARY_DIR}/Ryan_Debug_cmake-settings.h"
	)
	
configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/Ryan_Debug.xml.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.xml"
	)

# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

set( srcs
	src/config.cpp
	src/debug.cpp
	src/dlls.cpp
	src/error.cpp
	src/fs.cpp
	src/hash.cpp
	src/info.cpp
	src/io.cpp
	src/logging.cpp
	src/macros.cpp
	src/modules.cpp
	src/MurmurHash3.cpp
	src/os_functions.cpp
	src/registry.cpp
	src/serialization.cpp
	src/splitSet.cpp
	)
set (hdrs
	Ryan_Debug/config.h
	Ryan_Debug/defs.h
	Ryan_Debug/debug.h
	Ryan_Debug/dlls.h
	Ryan_Debug/error.h
	Ryan_Debug/fs.h
	Ryan_Debug/hash.h
	Ryan_Debug/info.h
	Ryan_Debug/io.h
	Ryan_Debug/logging.h
	Ryan_Debug/macros.h
	Ryan_Debug/modules.h
	Ryan_Debug/MurmurHash3.h
	Ryan_Debug/options.h
	Ryan_Debug/plugin.h
	Ryan_Debug/registry.h
	Ryan_Debug/Serialization.h
	Ryan_Debug/splitSet.h
	src/internal.h
	#	"${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h"
	"${CMAKE_BINARY_DIR}/Ryan_Debug_cmake-settings.h"
	"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.xml"
	)
source_group("Header Files" FILES ${hdrs})
source_group("Source Files" FILES ${srcs})
add_library(Ryan_Debug SHARED ${srcs} ${hdrs} 
	)
target_link_libraries(Ryan_Debug ${liblist})
addlib(Ryan_Debug SHARED)

add_subdirectory(plugins)
add_subdirectory(apps)




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
	if(WIN32 AND NOT CYGWIN) 
	# NSIS bug
	INSTALL(TARGETS Ryan_Debug
		RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
		COMPONENT Libraries
		#CONFIGURATIONS ${p}
		)

	# Regular windows

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
INSTALL(FILES
	#	"${MANIFEST_APP_SNIPPET_NAME}"
	"${CMAKE_BINARY_DIR}/Ryan_Debug_cmake-settings.h"
	#	"${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h"
	DESTINATION "${INSTALL_INCLUDE_DIR}/Ryan_Debug" COMPONENT Headers)

INSTALL(
		FILES "${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.xml"
		DESTINATION ${INSTALL_INCLUDE_DIR}/../share
		COMPONENT Scripts
	)

IF(INSTALL_MODULES)
	INSTALL(
		FILES ${PROJECT_BINARY_DIR}/module-ryan-debug
		DESTINATION ${ENV_MOD_DIR_PREFIX}
		COMPONENT Environment
		RENAME ${MODULES_FILENAME}
	)
ENDIF()

INSTALL(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan_DebugConfig.cmake"
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfigVersion.cmake"
	DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT Libraries)



