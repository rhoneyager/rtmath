include (SVNinfo)
include (signing)
include (manifest_append)

configure_file(logger.rc.in
	"${CMAKE_CURRENT_BINARY_DIR}/logger.rc" @ONLY)

add_executable (Ryan_Debug-logger 
	main.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/logger.rc")
set_target_properties( Ryan_Debug-logger PROPERTIES FOLDER "Apps")
#message("liblist: ${liblist}")
target_link_libraries(Ryan_Debug-logger ${liblist} Ryan_Debug)
if (MAKE_ASSEMBLY)
	#manifest_append(Ryan_Debug-logger ${MANIFEST_APP_SNIPPET_NAME})
	signing(Ryan_Debug-logger ${MANIFEST_APP_SNIPPET_NAME})
endif()

INSTALL(TARGETS Ryan_Debug-logger
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Applications
	)


