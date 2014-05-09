macro(addconfiginfo appname )
	if (MSVC)
		add_definitions(-DCONF="$(Configuration)${configappend}")
		add_definitions(-DBUILDTYPE=RYAN_DEBUG_BUILDTYPE_$(Configuration))
		#set(CONF CONF)
	endif()
	if (MINGW)
		add_definitions(-DCONF="${CMAKE_BUILD_TYPE}")
		add_definitions(-DBUILDTYPE=RYAN_DEBUG_BUILDTYPE_${CMAKE_BUILD_TYPE})
		set(CONF "\"${CMAKE_BUILD_TYPE}\"")
	endif()

	SET_TARGET_PROPERTIES( ${appname} PROPERTIES RELEASE_POSTFIX _Release${configappend} )
	SET_TARGET_PROPERTIES( ${appname} PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel${configappend} )
	SET_TARGET_PROPERTIES( ${appname} PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo${configappend} )
	SET_TARGET_PROPERTIES( ${appname} PROPERTIES DEBUG_POSTFIX _Debug${configappend})

endmacro(addconfiginfo appname )

