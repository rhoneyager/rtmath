macro(addInstallDirs basename version) 
	#MAJOR MINOR REVISION SVNREVISION INSTALL_BIN_DIR)
	#message("${MAJOR} ${MINOR} ${REVISION} ${SVNREVISION}")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for headers")
set(INSTALL_DOC_DIR share/doc/${basename}-${version} CACHE PATH "Installation directory for documentation")
if (MAKE_ASSEMBLY)
	set(INSTALL_ASSEMBLY_DIR ${INSTALL_BIN_DIR}/Ryan.${basename}.Release)
else()
	set(INSTALL_ASSEMBLY_DIR ${INSTALL_BIN_DIR})
endif()

if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake/conf${configappend})
else()
	set(DEF_INSTALL_CMAKE_DIR lib/CMake/${basename}/conf${configappend})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
	"Installation directory for CMake files")

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

endmacro(addInstallDirs basename version)

