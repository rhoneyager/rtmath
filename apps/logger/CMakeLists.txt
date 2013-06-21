add_executable (Ryan_Debug-logger main.cpp)
set_target_properties( Ryan_Debug-logger PROPERTIES FOLDER "Apps")
#message("liblist: ${liblist}")
target_link_libraries(Ryan_Debug-logger ${liblist} Ryan_Debug)
INSTALL(TARGETS Ryan_Debug-logger
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Applications
	)


