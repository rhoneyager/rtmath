include (SVNinfo)

configure_file(test.rc.in
	"${CMAKE_CURRENT_BINARY_DIR}/test.rc" @ONLY)

add_executable (Ryan_Debug-test
	test.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/test.rc")
set_target_properties( Ryan_Debug-test PROPERTIES FOLDER "Apps")
#message("liblist: ${liblist}")
target_link_libraries(Ryan_Debug-test ${liblist} Ryan_Debug)
INSTALL(TARGETS Ryan_Debug-test
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Applications
	)


