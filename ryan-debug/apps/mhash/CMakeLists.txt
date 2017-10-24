include (SVNinfo)
include (signing)
include (addconfiginfo)
include (addlib)

configure_file(mhash.rc.in
	"${CMAKE_CURRENT_BINARY_DIR}/mhash.rc" @ONLY)

if (MAKE_ASSEMBLY)
	set(addfiles "${MANIFEST_APP_SNIPPET_NAME}")
else()
	set(addfiles)
endif()
add_executable (Ryan_Debug-mhash
	main.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/mhash.rc"
	${addfiles})
set_target_properties( Ryan_Debug-mhash PROPERTIES FOLDER "Apps")
addconfiginfo(Ryan_Debug-mhash)
target_link_libraries(Ryan_Debug-mhash ${liblist} Ryan_Debug)
if (MAKE_ASSEMBLY)
	signing(Ryan_Debug-mhash)
endif()

INSTALL(TARGETS Ryan_Debug-mhash
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	COMPONENT Applications
	)
storebin(Ryan_Debug-mhash)

