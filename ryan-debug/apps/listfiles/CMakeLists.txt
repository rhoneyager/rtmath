include (common)
include (commonBaseApps)

include (SVNinfo)
include (signing)
include (addconfiginfo)
include (addlib)

set (srcs main.cpp)
if(WIN32 AND NOT CYGWIN)
	generaterc(Ryan_Debug-listfiles dll Ryan_Debug "Install-time program to generate file listings and scripts for windows" "${CMAKE_CURRENT_BINARY_DIR}/app.rc")
	set (srcs ${srcs} "${CMAKE_CURRENT_BINARY_DIR}/app.rc" )
endif()

add_executable (Ryan_Debug-listfiles
	${srcs})
	addconfiginfo(Ryan_Debug-listfiles)
set_target_properties( Ryan_Debug-listfiles PROPERTIES FOLDER "Apps")
addconfiginfo(Ryan_Debug-listfiles)
target_link_libraries(Ryan_Debug-listfiles ${Boost_LIBRARIES})

storebin(Ryan_Debug-listfiles)

