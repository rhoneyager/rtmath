include (common)
include (commonBaseApps)

set (srcs plugin-test.cpp
	plugin-test.h)
if(WIN32 AND NOT CYGWIN)
	generaterc(Ryan_Debug_plugin_test dll Ryan_Debug "Test plugin" "${CMAKE_CURRENT_BINARY_DIR}/lib.rc")
	set (srcs ${srcs} "${CMAKE_CURRENT_BINARY_DIR}/lib.rc" )
endif()


add_library (Ryan_Debug_plugin_test SHARED 
	${srcs}
	)
addlib(Ryan_Debug_plugin_test SHARED)

addplugin(Ryan_Debug_plugin_test Testing)

target_link_libraries(Ryan_Debug_plugin_test ${liblist} Ryan_Debug )

