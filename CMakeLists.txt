# CMake file for Ryan's Debugging library
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
include(generaterc)

set(MAJOR 0)
set(MINOR 10)
set(REVISION 0)

set(RYAN_DEBUG_VERSION
	${MAJOR}.${MINOR}.${REVISION}.${SVNREVISION})

math(EXPR PREVSVNREVISION "${SVNREVISION} - 1")

addBaseProject()
addInstallDirs(Ryan_Debug ${RYAN_DEBUG_VERSION})
addModuleFiles(Ryan_Debug ${RYAN_DEBUG_VERSION})
addAssembly(Ryan_Debug)


# Needed here to properly configure
set(COMPRESS_ZLIB 0)
set(COMPRESS_GZIP 0)
set(COMPRESS_BZIP2 0)
set(COMPRESS_SZIP 0) # Only the HDF plugin uses SZIP

find_package(BZip2)
# BZIP2_FOUND requires that the debug libary be found, which is not desired.
if (NOT BZIP2_INCLUDE_DIR)
	message(STATUS "BZip2 libraries not found. Retrying with environment variables.")
	# FindBZip.cmake is annoying. It doesn't check the environment.
	if (DEFINED ENV{BZIP2_INCLUDE_DIR})
		set(BZIP2_INCLUDE_DIR $ENV{BZIP2_INCLUDE_DIR}
			CACHE PATH "BZip2 Include directory" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARY_DEBUG})
		set(BZIP2_LIBRARY_DEBUG $ENV{BZIP2_LIBRARY_DEBUG}
			CACHE FILEPATH "BZip2 Debug library path" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARY_RELEASE})
		set(BZIP2_LIBRARY_RELEASE $ENV{BZIP2_LIBRARY_RELEASE}
			CACHE FILEPATH "BZip2 Release library path" FORCE)
	endif()
	if (DEFINED ENV{BZIP2_LIBRARIES})
		set(BZIP2_LIBRARIES $ENV{BZIP2_LIBRARIES}
			CACHE FILEPATH "BZip2 libraries" FORCE)
		mark_as_advanced(BZIP2_LIBRARIES)
	endif()
	find_package(BZip2)
endif()


if(NOT ZLIB_INCLUDE_DIR)
	if (DEFINED ENV{ZLIB_ROOT})
		set(ZLIB_ROOT $ENV{ZLIB_ROOT})
	ENDIF()
endif()
find_package(ZLIB)


if (ZLIB_FOUND)
	set(COMPRESS_ZLIB 1)
	set(COMPRESS_GZIP 1)
	include_directories(${ZLIB_INCLUDE_DIRS})
	#set(liblist ${liblist} ${ZLIB_LIBRARIES})
endif()
# BZIP2_FOUND requires that the debug libary be found, which is not desired.
if (BZIP2_INCLUDE_DIR)
	set(COMPRESS_BZIP2 1)
	include_directories(${BZIP2_INCLUDE_DIR})
	#set(liblist ${liblist} ${BZIP2_LIBRARIES})
endif()
IF (DEFINED SZIP_LIBRARY)
	include_directories(${SZIP_INCLUDE_DIRS})
	#	SET (h5libs ${h5libs} szip)
	set(COMPRESS_SZIP 1)
ENDIF()

install (SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/CMakeRules/implSigning.cmake")
# Configure the subversion.h file
configure_file(
	"debug_subversion.h.template"
	"${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h" @ONLY)


configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake-settings.h.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h"
	)
	
configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/Ryan_Debug.xml.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.xml"
	)

# Add in script-generated includes
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

#configure_file(src/lib.rc.in
#	"${PROJECT_BINARY_DIR}/lib.rc" @ONLY)
#configure_file(src/versioninfo.h.in
#	"${PROJECT_BINARY_DIR}/versioninfo.h" @ONLY)

addBoostUniform(filesystem iostreams program_options system log log_setup)

include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

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
	"${CMAKE_CURRENT_BINARY_DIR}/debug_subversion.h"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h"
	"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.xml"
	)

#set(srcs-test src/test.cpp)

if(WIN32 AND NOT CYGWIN)
	generaterc(Ryan_Debug dll Ryan_Debug "Core debugging library" "${PROJECT_BINARY_DIR}/lib.rc")
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
#if (NOT DEFINED MAKE_ASSEMBLY)
#	signing(Ryan_Debug) # Will be part of the assembly code
#endif()

if (MAKE_ASSEMBLY)
	set (USINGMANIFESTS 1)
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
else()
	set (USINGMANIFESTS 0)
endif()
if(WIN32 AND NOT CYGWIN)
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.template.wxs"
		"${CMAKE_CURRENT_BINARY_DIR}/config${configappend}.wxs" @ONLY)
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/Ryan_Debug.template.wxs"
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.wxs" @ONLY)

	

endif()
# App snippets
set (MANIFEST_APP_SNIPPET_NAME "${CMAKE_BINARY_DIR}/Ryan.Ryan_Debug.manifest.h")
	configure_file(
		"manifestref.h.in"
		"${MANIFEST_APP_SNIPPET_NAME}" @ONLY)
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
	"${MANIFEST_APP_SNIPPET_NAME}"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake-settings.h"
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

adddocs()

INSTALL(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/Ryan_DebugConfig.cmake"
	"${PROJECT_BINARY_DIR}/Ryan_DebugConfigVersion.cmake"
	DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT Libraries)


#message("Delayed signing: ${dsfiles}")

