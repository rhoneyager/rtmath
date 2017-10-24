option (BUILD_PLUGINS
	"Master switch to enable / disable building of plugins" ON)
if (BUILD_PLUGINS)
	option (BUILD_PLUGIN_TEST
		"Build the test plugin" ON)
	if (BUILD_PLUGIN_TEST)
		add_subdirectory(test)
	endif()
endif()

