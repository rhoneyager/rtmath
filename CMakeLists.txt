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
set(MINOR 7)
set(REVISION 0)

set(RYAN_DEBUG_VERSION
	${MAJOR}.${MINOR}.${REVISION}.${SVNREVISION})

math(EXPR PREVSVNREVISION "${SVNREVISION} - 1")

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
	set (usesmodules OFF)
	set (modinstall OFF)
else()
	set (usesmodules ON)
	set (modinstall ON)
endif()

if(usesmodules)
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
endif()

# Assembly manifest information
if (WIN32 AND NOT CYGWIN)
	option (MAKE_ASSEMBLY "Create an application assembly" ON)
	set(ASSEMBLY_PUBLICKEYTOKEN
		"ca89aae88144abf2"
		CACHE STRING
		"Public id of signing key (defaulting to Liulab-3 CA)"
		)
	set (ASSEMBLY_SIGN ON)
	#option (ASSEMBLY_SIGN
	#	"Sign assembly and all binaries" ON)

	# Default install directory is under bin/.
	# Directory is designed to be copy-ready, for easy private deployment.
	# Example app manifest include snippet is under this directory, in sampleapp.manifest.
	set(ASSEMBLY_MINVERSION
		"${MAJOR}.${MINOR}.0.0"
		CACHE STRING
		"Specify the minimum version that this assembly can override (used when assembly is shared)")

	set(INSTALL_ASSEMBLY_DIR ${INSTALL_BIN_DIR}/Ryan.Debug.DebugAssembly)
else()
	set(MAKE_ASSEMBLY OFF)
	set(INSTALL_ASSEMBLY_DIR ${INSTALL_BIN_DIR})
	set(ASSEMBLY_SIGN OFF)
endif()

set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
	"Installation directory for CMake files")

if (WIN32 AND NOT CYGWIN)
# TODO: Add logic when creating a Wix installer to include and specialize raw cmake 
# files with string substitution for appropriate install directories.
# TODO: Add configurable Wix installer script.
# TODO: Add signtool support in Wix script
endif()

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
#foreach(p LIB BIN INCLUDE CMAKE DOC ASSEMBLY)
#	set(var INSTALL_${p}_DIR)
#	if(NOT IS_ABSOLUTE "${${var}}")
#		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
#	endif()
#endforeach()
 
configure_file (
	"${CMAKE_CURRENT_SOURCE_DIR}/environment/ryan-debug"
	"${PROJECT_BINARY_DIR}/module-ryan-debug"
	)


find_package(Boost COMPONENTS filesystem program_options system REQUIRED)
include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
if (WIN32 AND NOT CYGWIN)
	option ( AUTOLINK_BOOST
		"Automatically link Boost" ON)
	if (AUTOLINK_BOOST)
		link_directories(${Boost_LIBRARY_DIR})
	else()
		add_definitions(-DBOOST_ALL_NO_LIB)
		set(liblist ${Boost_LIBRARIES})
	endif()
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

if(WIN32 AND NOT CYGWIN)
	set (srcs ${srcs} "${PROJECT_BINARY_DIR}/lib.rc" )
endif()

source_group("Header Files" FILES ${hdrs})
source_group("Source Files" FILES ${srcs})

set(configappend "")
if(MSVC)
	#message( STATUS "CMAKE_CL_64: ${CMAKE_CL_64}")
	if (CMAKE_CL_64)
		set(configappend "_x64")
	else()
		set(configappend "_x86")
	endif()
	add_definitions(-DCONF="${configappend}")
	#add_definitions(-DCONF="$(Configuration)${configappend}")
	#set(CONF "\"$(Configuration)\"")
	set(CONF CONF)
endif()

add_library(Ryan_Debug SHARED ${srcs} ${hdrs} 
	"${PROJECT_BINARY_DIR}/module-ryan-debug"
	)
target_link_libraries(Ryan_Debug ${liblist})
set_target_properties(Ryan_Debug PROPERTIES FOLDER "Libs")
if (configappend)
	SET_TARGET_PROPERTIES( Ryan_Debug PROPERTIES RELEASE_POSTFIX ${configappend} )
	SET_TARGET_PROPERTIES( Ryan_Debug PROPERTIES MINSIZEREL_POSTFIX ${configappend} )
	SET_TARGET_PROPERTIES( Ryan_Debug PROPERTIES RELWITHDEBINFO_POSTFIX ${configappend} )
	SET_TARGET_PROPERTIES( Ryan_Debug PROPERTIES DEBUG_POSTFIX ${configappend} )
endif()

if (MAKE_ASSEMBLY)
	# Install script
	configure_file("Ryan_Debug.template.wxs"
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.wxs" @ONLY)

	# Main DLL
	configure_file(
		"Ryan.Debug.DebugAssembly.manifest.template"
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.pre.manifest" @ONLY)

	# Alternate SxS assembly formulation - the full manifest is embedded into the dll
	configure_file(
		"Ryan.Debug.DebugAssembly.manifest.single.template"
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.single.pre.manifest" @ONLY)

	# Shared assembly redirect
	set (PublisherManifestName "policy.@MAJOR@.@MINOR@.Ryan.Debug.DebugAssembly.manifest")
	configure_file(
		"Ryan.Debug.DebugAssembly.manifest.publisher.template"
		"${CMAKE_CURRENT_BINARY_DIR}/${PublisherManifestName}" @ONLY)

	# App snippets
	set (MANIFEST_APP_SNIPPET_NAME "${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.manifest.h")
	configure_file(
		"src/manifestref.h.in"
		"${MANIFEST_APP_SNIPPET_NAME}" @ONLY)

	
	# Sign the dll file and the apps
	add_custom_command(TARGET Ryan_Debug POST_BUILD
		COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll $<TARGET_FILE:Ryan_Debug>
		COMMENT "Signing the library"
		)
	
	#add_custom_target(Ryan_Debug_Manifest_Hashing_test
	#	COMMAND echo mt.exe -manifest "${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.pre.manifest" 
	#	-hashupdate:"$<TARGET_FILE_DIR:Ryan_Debug>" -makecdfs -out:"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.manifest" 
	#	DEPENDS Ryan_Debug
	#	COMMENT "Adding dll assembly hash information"
	#	)

	# Fill in the hashes in the dll assembly
	add_custom_target(Ryan_Debug_Manifest_Hashing
		COMMAND mt.exe -manifest Ryan.Debug.DebugAssembly.pre.manifest 
		-hashupdate:"$<TARGET_FILE_DIR:Ryan_Debug>" -makecdfs -out:Ryan.Debug.DebugAssembly.manifest
		DEPENDS Ryan_Debug
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Adding dll assembly hash information"
		)

	# Make the assembly catalog
	add_custom_target(Ryan_Debug_Create_Catalog
		COMMAND makecat.exe Ryan.Debug.DebugAssembly.manifest.cdf
		DEPENDS Ryan_Debug_Manifest_Hashing
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Making the assembly catalog"
		)

	# Sign the assembly catalog
	add_custom_target(Ryan_Debug_Sign_Catalog ALL
		COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll 
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.cat"
		DEPENDS Ryan_Debug_Create_Catalog
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Signing the assembly catalog"
		)

	# Create the policy cdf
	add_custom_target(Ryan_Debug_Policy_Hashing
		COMMAND mt.exe -manifest ${PublisherManifestName} -makecdfs
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Adding policy hash information"
		)

	# Make the policy catalog
	add_custom_target(Ryan_Debug_Create_Policy_Catalog
		COMMAND makecat.exe policy.@MAJOR@.@MINOR@.Ryan.Debug.DebugAssembly.manifest.cdf
		DEPENDS Ryan_Debug_Policy_Hashing
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Making the policy catalog"
		)

	# Sign the policy catalog
	add_custom_target(Ryan_Debug_Sign_Policy_Catalog ALL
		COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll 
		policy.@MAJOR@.@MINOR@.Ryan.Debug.DebugAssembly.cat
		DEPENDS Ryan_Debug_Create_Policy_Catalog
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Signing the policy catalog"
		)

	# Add installation targets for the library manifest, the sample app manifest and the library catalog
	INSTALL(FILES 
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.cat"
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.manifest"
		"${CMAKE_CURRENT_BINARY_DIR}/${PublisherManifestName}"
		"${CMAKE_CURRENT_BINARY_DIR}/policy.@MAJOR@.@MINOR@.Ryan.Debug.DebugAssembly.cat"
		#"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.app.manifest"
		DESTINATION "${INSTALL_ASSEMBLY_DIR}" COMPONENT Libraries)

	INSTALL(FILES
		"${MANIFEST_APP_SNIPPET_NAME}"
		DESTINATION "${INSTALL_INCLUDE_DIR}/Ryan_Debug" COMPONENT Headers)

	INSTALL(FILES
		"${CMAKE_CURRENT_BINARY_DIR}/Ryan_Debug.wxs"
		DESTINATION "${INSTALL_INCLUDE_DIR}/.." COMPONENT Scripts)

	# The single-dll form of the assembly is generated here...
	#add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.dll"
	#	COMMAND copy "$<TARGET_FILE:Ryan_Debug>" "${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.dll"
	#	DEPENDS Ryan_Debug
	#	)
	#add_custom_target(Ryan_Debug_Embed_Manifest_Single
	#	COMMAND mt.exe -manifest "Ryan.Debug.DebugAssembly.single.pre.manifest" 
	#	-outputresource:"Ryan.Debug.DebugAssembly.dll"\;\#1
	#	DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.dll"
	#	COMMENT "Embedding single dll-version manifest into dll"
	#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	#	)
	# Sign the single dll
	#add_custom_target(Ryan_Debug_Sign_DLL_Single
	#	COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll 
	#	"Ryan.Debug.DebugAssembly.dll"
	#	DEPENDS Ryan_Debug_Embed_Manifest_Single
	#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	#	COMMENT "Signing the single dll form"
	#	)
	# Extract the assembly catalog
	#add_custom_target(Ryan_Debug_Manifest_Single_Extract
	#	COMMAND mt.exe -inputresource "Ryan.Debug.DebugAssembly.dll"\;\#1
	#	-hashupdate:"Ryan.Debug.DebugAssembly.dll" -makecdfs 
	#	DEPENDS Ryan_Debug_Embed_Manifest_Single
	#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	#	COMMENT "Adding dll assembly hash information"
	#	)
	# Make the assembly catalog
	#add_custom_target(Ryan_Debug_Create_Catalog_Single
	#	COMMAND makecat.exe Ryan.Debug.DebugAssembly.cdf
	#	DEPENDS Ryan_Debug_Manifest_Single_Extract
	#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	#	COMMENT "Making the assembly catalog"
	#	)
	# Sign the assembly catalog
	#add_custom_target(Ryan_Debug_Sign_Catalog_Single
	#	COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll 
	#	"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.cat"
	#	DEPENDS Ryan_Debug_Manifest_Single_Extract
	#	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	#	COMMENT "Signing the catalog"
	#	)
	# Add installation targets for the library manifest
	#INSTALL(FILES
	#	"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.DebugAssembly.dll"
	#	DESTINATION "${INSTALL_BIN_DIR}" COMPONENT Libraries
	#	)

endif()

add_subdirectory(apps)

#add_executable(Ryan_Debug-test ${srcs-test})
#target_link_libraries(Ryan_Debug-test ${liblist} Ryan_Debug)
#set_target_properties(Ryan_Debug-test PROPERTIES FOLDER "Apps")

export(TARGETS Ryan_Debug
	FILE "${PROJECT_BINARY_DIR}/Ryan_DebugTargets.cmake")
export(PACKAGE Ryan_Debug)


# Constructing relative and absolute paths, needed for the cmake export file header locator
# Absolute paths: ABS_INSTALL_${p}_DIR
# Relative paths: REL_${p}_DIR
foreach(p LIB BIN INCLUDE CMAKE DOC ASSEMBLY)
	set(var ABS_INSTALL_${p}_DIR)
	set(ABS_INSTALL_${p}_DIR ${INSTALL_${p}_DIR})
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
	#message("ABS_INSTALL_${p}_DIR - ${${var}}")
endforeach()
foreach(p LIB BIN INCLUDE DOC ASSEMBLY)
	set(var ABS_INSTALL_${p}_DIR)
	#message("RELATIVE_PATH REL_${p}_DIR ${ABS_INSTALL_CMAKE_DIR} ${${var}}")
	file(RELATIVE_PATH REL_${p}_DIR "${ABS_INSTALL_CMAKE_DIR}" "${${var}}")
	#message("      - ${REL_${p}_DIR}")
endforeach()

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
if(WIN32 AND NOT CYGWIN) # NSIS bug
INSTALL(TARGETS Ryan_Debug
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_ASSEMBLY_DIR}
	COMPONENT Libraries
	)
endif()

INSTALL(TARGETS Ryan_Debug
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_ASSEMBLY_DIR}
	LIBRARY DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}
	ARCHIVE DESTINATION ${INSTALL_CMAKE_DIR}/${REL_LIB_DIR}
	COMPONENT Libraries
	)

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


