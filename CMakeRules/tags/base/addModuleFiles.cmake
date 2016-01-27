macro(addModuleFiles basename version) 
	#MAJOR MINOR REVISION SVNREVISION INSTALL_BIN_DIR)
	#message("${MAJOR} ${MINOR} ${REVISION} ${SVNREVISION}")
if(WIN32 AND NOT CYGWIN)
	set (usesmodules OFF)
	set (modinstall OFF)
else()
	set (usesmodules ON)
	set (modinstall ON)
endif()
if (usesmodules)
	option ( INSTALL_MODULES
		"Install the environment-modules package file. Does nothing on Windows." ${modinstall})

	SET( ENV_MOD_DIR_PREFIX
		/etc/modulefiles/${basename}
		CACHE STRING
		"Environment-modules package file install directory"
	)

	set(MODULES_FILENAME ${version}${configappend})

	SET( CMAKE_INSTALL_PREFIX_SUGGESTED 
		"/opt/${basename}/${version}"
		CACHE STRING
		"Suggested prefix"
	)
endif()

endmacro(addModuleFiles basename version)

