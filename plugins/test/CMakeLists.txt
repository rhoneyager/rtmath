include (common)
include (commonBaseApps)

add_library (Ryan_Debug_plugin_test SHARED 
	plugin-test.cpp
	plugin-test.h
	)
addlib(Ryan_Debug_plugin_test SHARED)

addplugin(Ryan_Debug_plugin_test Testing)

target_link_libraries(Ryan_Debug_plugin_test ${liblist} Ryan_Debug )

