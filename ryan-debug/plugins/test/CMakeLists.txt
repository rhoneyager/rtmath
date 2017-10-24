include (common)
include (commonBaseApps)

set (srcs plugin-test.cpp
	plugin-test.h)
if(WIN32 AND NOT CYGWIN)
	generaterc(plugin_test dll Ryan_Debug "Test plugin" "${CMAKE_CURRENT_BINARY_DIR}/lib.rc")
	set (srcs ${srcs} "${CMAKE_CURRENT_BINARY_DIR}/lib.rc" )
endif()


add_library (plugin_test SHARED 
	${srcs}
	)
addlib(plugin_test SHARED)

addplugin(plugin_test Testing Ryan_Debug)

target_link_libraries(plugin_test ${liblist} Ryan_Debug )

