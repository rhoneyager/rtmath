if (BUILD_PLUGINS)
	option (BUILD_PLUGIN_RYAN_DEBUG_TEST
		"Build the test plugin" ON)
	if (BUILD_PLUGIN_RYAN_DEBUG_TEST)
		add_subdirectory(test)
	endif()
endif()

