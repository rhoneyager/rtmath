include (SVNinfo)
include (signing)
include (addconfiginfo)
include (addlib)

configure_file(splitset.rc.in
	"${CMAKE_CURRENT_BINARY_DIR}/splitset.rc" @ONLY)

if (MAKE_ASSEMBLY)
	set(addfiles "${MANIFEST_APP_SNIPPET_NAME}")
else()
	set(addfiles)
endif()
add_executable (Ryan_Debug-splitset
	main.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/splitset.rc"
	${addfiles})
set_target_properties( Ryan_Debug-splitset PROPERTIES FOLDER "Apps")
addconfiginfo(Ryan_Debug-splitset)
#message("liblist: ${liblist}")
target_link_libraries(Ryan_Debug-splitset ${liblist} Ryan_Debug)
if (MAKE_ASSEMBLY)
	signing(Ryan_Debug-splitset)
endif()

INSTALL(TARGETS Ryan_Debug-splitset
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	COMPONENT Applications
	)
storebin(Ryan_Debug-splitset)

