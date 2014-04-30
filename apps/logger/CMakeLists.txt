include (SVNinfo)
include (signing)

configure_file(logger.rc.in
	"${CMAKE_CURRENT_BINARY_DIR}/logger.rc" @ONLY)

if (MAKE_ASSEMBLY)
	set(addfiles "${MANIFEST_APP_SNIPPET_NAME}")
else()
	set(addfiles)
endif()
add_executable (Ryan_Debug-logger 
	main.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/logger.rc"
	${addfiles})
set_target_properties( Ryan_Debug-logger PROPERTIES FOLDER "Apps")
#message("liblist: ${liblist}")
target_link_libraries(Ryan_Debug-logger ${liblist} Ryan_Debug)
if (MAKE_ASSEMBLY)
	signing(Ryan_Debug-logger)
endif()

INSTALL(TARGETS Ryan_Debug-logger
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Applications
	)


