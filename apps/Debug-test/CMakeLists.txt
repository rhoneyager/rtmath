include (SVNinfo)
include (signing)

configure_file(test.rc.in
	"${CMAKE_CURRENT_BINARY_DIR}/test.rc" @ONLY)
if (MAKE_ASSEMBLY)
	set(addfiles "${MANIFEST_APP_SNIPPET_NAME}")
else()
	set(addfiles)
endif()
add_executable (Ryan_Debug-test
	test.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/test.rc"
	${addfiles})
set_target_properties( Ryan_Debug-test PROPERTIES FOLDER "Apps")
target_link_libraries(Ryan_Debug-test ${liblist} Ryan_Debug)
if (MAKE_ASSEMBLY)
	signing(Ryan_Debug-test)
endif()

INSTALL(TARGETS Ryan_Debug-test
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	COMPONENT Applications
	)


