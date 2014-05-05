macro(addAssembly basename ) 
	#MAJOR MINOR REVISION SVNREVISION INSTALL_BIN_DIR)
	#message("${MAJOR} ${MINOR} ${REVISION} ${SVNREVISION}")
# Assembly manifest information
if (WIN32 AND NOT CYGWIN)
	option (MAKE_ASSEMBLY "Create an application assembly" ON)
	set(ASSEMBLY_NAME_Debug
		"Ryan.${basename}.Debug"
		CACHE STRING
		"Assembly name for debug builds")
	set(ASSEMBLY_NAME_RelWithDebInfo
		"Ryan.${basename}.RelWithDebInfo"
		CACHE STRING
		"Assembly name for RelWithDebInfo builds")
	set(ASSEMBLY_NAME_Release
		"Ryan.${basename}.Release"
		CACHE STRING
		"Assembly name for release builds")
	set(ASSEMBLY_NAME_MinSizeRel
		"Ryan.${basename}.MinSizeRel"
		CACHE STRING
		"Assembly name for MinSizeRel builds")
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

