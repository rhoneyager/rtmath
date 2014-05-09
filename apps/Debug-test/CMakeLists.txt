include (SVNinfo)
include (signing)
include (addconfiginfo)

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
addconfiginfo(Ryan_Debug-test)
target_link_libraries(Ryan_Debug-test ${liblist} Ryan_Debug)
if (MAKE_ASSEMBLY)
	signing(Ryan_Debug-test)
endif()

INSTALL(TARGETS Ryan_Debug-test
	EXPORT Ryan_DebugTargets
	RUNTIME DESTINATION ${INSTALL_CMAKE_DIR}/${REL_BIN_DIR}/bin${configappend}
	COMPONENT Applications
	)


