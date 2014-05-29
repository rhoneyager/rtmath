include (SVNinfo)
include (signing)
include (addconfiginfo)

configure_file(module.rc.in
	"${CMAKE_CURRENT_BINARY_DIR}/module.rc" @ONLY)

if (MAKE_ASSEMBLY)
	set(addfiles "${MANIFEST_APP_SNIPPET_NAME}")
else()
	set(addfiles)
endif()
add_executable (Ryan_Debug-module 
	main.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/module.rc"
	${addfiles})
set_target_properties( Ryan_Debug-module PROPERTIES FOLDER "Apps")
addconfiginfo(Ryan_Debug-module)
#message("liblist: ${liblist}")
target_link_libraries(Ryan_Debug-module ${liblist} Ryan_Debug)
if (MAKE_ASSEMBLY)
	signing(Ryan_Debug-module)
endif()

INSTALL(TARGETS Ryan_Debug-module
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	COMPONENT Applications
	)


