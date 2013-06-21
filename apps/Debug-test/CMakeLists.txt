add_executable (Ryan_Debug-test test.cpp)
set_target_properties( Ryan_Debug-test PROPERTIES FOLDER "Apps")
#message("liblist: ${liblist}")
target_link_libraries(Ryan_Debug-test ${liblist} Ryan_Debug)
INSTALL(TARGETS Ryan_Debug-test
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Applications
	)


