include (signing)

macro(addAssembly basename ) 
	#MAJOR MINOR REVISION SVNREVISION INSTALL_BIN_DIR)
	#message("${MAJOR} ${MINOR} ${REVISION} ${SVNREVISION}")
# Assembly manifest information
if (WIN32 AND NOT CYGWIN)
	option (MAKE_ASSEMBLY "Create an application assembly" ON)
	set(ASSEMBLY_NAME_Debug
		"Ryan.${basename}.Debug")
		#CACHE STRING
		#"Assembly name for debug builds")
	set(ASSEMBLY_NAME_RelWithDebInfo
		"Ryan.${basename}.RelWithDebInfo")
		#CACHE STRING
		#"Assembly name for RelWithDebInfo builds")
	set(ASSEMBLY_NAME_Release
		"Ryan.${basename}.Release")
		#CACHE STRING
		#"Assembly name for release builds")
	set(ASSEMBLY_NAME_MinSizeRel
		"Ryan.${basename}.MinSizeRel")
		#CACHE STRING
		#"Assembly name for MinSizeRel builds")
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

	set(INSTALL_ASSEMBLY_DIR ${INSTALL_BIN_DIR}/Ryan.${basename}.Release)
else()
	set(MAKE_ASSEMBLY OFF)
	set(INSTALL_ASSEMBLY_DIR ${INSTALL_BIN_DIR})
	set(ASSEMBLY_SIGN OFF)
endif()
 
endmacro(addAssembly basename )

macro(implementAssembly basename targetname ) #packagein)
	# Set dll properties for assembly name
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES Release_ASSEMBLY ${ASSEMBLY_NAME_Release} )
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES MinSizeRel_ASSEMBLY ${ASSEMBLY_NAME_MinSizeRel} )
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES RelWithDebInfo_ASSEMBLY ${ASSEMBLY_NAME_RelWithDebInfo} )
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES Debug_ASSEMBLY ${ASSEMBLY_NAME_Debug} )

	# Install script
	#configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${packagein}"
	#	"${CMAKE_CURRENT_BINARY_DIR}/${packagein}" @ONLY)

	# Main DLL

	# TODO: reenable when more config is added
	#foreach(p Debug RelWithDebInfo MinSizeRel Release)
	#	set(ASSEMBLY_NAME ${ASSEMBLY_NAME_${p}})
	#	configure_file(
	#		"Ryan.${basename}.Release.manifest.template"
	#		"${CMAKE_CURRENT_BINARY_DIR}/${p}/Ryan.${basename}.Release.pre.manifest" @ONLY)
	#endforeach()

	
	# Alternate SxS assembly formulation - the full manifest is embedded into the dll
	#configure_file(
	#	"Ryan.Debug.Release.manifest.single.template"
	#	"${CMAKE_CURRENT_BINARY_DIR}/Ryan.Debug.Release.single.pre.manifest" @ONLY)

	# Shared assembly redirect
	#set (PublisherManifestName "policy.@MAJOR@.@MINOR@.Ryan.${basename}.Release.manifest")
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES Release_PUBLISHER "policy.@MAJOR@.@MINOR@.${ASSEMBLY_NAME_Release}.manifest" )
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES MinSizeRel_PUBLISHER "policy.@MAJOR@.@MINOR@.${ASSEMBLY_NAME_MinSizeRel}.manifest" )
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES RelWithDebInfo_PUBLISHER "policy.@MAJOR@.@MINOR@.${ASSEMBLY_NAME_RelWithDebInfo}.manifest")
	SET_TARGET_PROPERTIES( ${targetname} PROPERTIES Debug_PUBLISHER "policy.@MAJOR@.@MINOR@.${ASSEMBLY_NAME_Debug}.manifest" )
	#configure_file(
	#	"Ryan.${basename}.Release.manifest.publisher.template"
	#	"${CMAKE_CURRENT_BINARY_DIR}/${PublisherManifestName}" @ONLY)
	# TODO: For each configuration, configure a SEPARATE policy file!
	foreach(p Debug RelWithDebInfo MinSizeRel Release)
		configure_file(
			"Ryan.${basename}.Release.manifest.publisher.template"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/policy.@MAJOR@.@MINOR@.${ASSEMBLY_NAME_${p}}.manifest" @ONLY)
	endforeach()	

	# App snippets
	set (MANIFEST_APP_SNIPPET_NAME "${CMAKE_CURRENT_BINARY_DIR}/Ryan.${basename}.Release.manifest.h")
	configure_file(
		"manifestref.h.in"
		"${MANIFEST_APP_SNIPPET_NAME}" @ONLY)

	
	# Sign the dll file and the apps
	add_custom_command(TARGET ${targetname} POST_BUILD
		COMMAND echo $<CONFIGURATION> - Signing DLL
		COMMAND cd $<CONFIGURATION>
		COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll $<TARGET_FILE:${targetname}>
		COMMAND echo Updating manifest with hashes and making CDF
		COMMAND mt.exe -manifest $<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.pre.manifest
			-hashupdate:.
			-makecdfs 
			-out:$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.manifest
		COMMAND echo Making catalog
		COMMAND echo makecat.exe -v $<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.manifest.cdf
		COMMAND makecat.exe -v $<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.manifest.cdf
		COMMAND echo Signing catalog
		COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll 
			 "$<TARGET_FILE_DIR:${targetname}>/$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.cat"
		COMMAND echo Creating the policy cdf
		COMMAND echo mt.exe -manifest policy.@MAJOR@.@MINOR@.$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.manifest -makecdfs
		COMMAND mt.exe -manifest policy.@MAJOR@.@MINOR@.$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.manifest -makecdfs 
		COMMAND echo Make the policy catalog
		COMMAND echo makecat.exe policy.@MAJOR@.@MINOR@.$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.manifest.cdf
		COMMAND makecat.exe policy.@MAJOR@.@MINOR@.$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.manifest.cdf
		COMMAND echo Sign the policy catalog
		COMMAND echo signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll 
			policy.@MAJOR@.@MINOR@.$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.cat
		COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll 
			policy.@MAJOR@.@MINOR@.$<TARGET_PROPERTY:${targetname},$<CONFIGURATION>_ASSEMBLY>.cat
		COMMAND cd ..
		)

	foreach(p Debug RelWithDebInfo MinSizeRel Release)
		INSTALL(FILES
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/${ASSEMBLY_NAME_${p}}.cat"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/${ASSEMBLY_NAME_${p}}.manifest"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/policy.@MAJOR@.@MINOR@.${ASSEMBLY_NAME_${p}}.manifest"
			"${CMAKE_CURRENT_BINARY_DIR}/${p}/policy.@MAJOR@.@MINOR@.${ASSEMBLY_NAME_${p}}.cat"
			DESTINATION "${INSTALL_BIN_DIR}/${ASSEMBLY_NAME_${p}}${configappend}"
			CONFIGURATIONS ${p}
			COMPONENT Libraries
			)
	endforeach()	

	INSTALL(FILES
		"${MANIFEST_APP_SNIPPET_NAME}"
		DESTINATION "${INSTALL_INCLUDE_DIR}/${basename}" COMPONENT Headers)

	#INSTALL(FILES
	#	"${CMAKE_CURRENT_BINARY_DIR}/${packagein}"
	#	"${CMAKE_CURRENT_SOURCE_DIR}/upgrade.bat"
	#	"${CMAKE_CURRENT_SOURCE_DIR}/deploy.bat"
	#	DESTINATION "${INSTALL_INCLUDE_DIR}/.." COMPONENT Scripts)

endmacro(implementAssembly basename targetname ) #packagein)

