macro(addBaseProject)

# Enable C++11
# g++
IF(DEFINED CMAKE_COMPILER_IS_GNUCXX)
	SET (COMMON_CFLAGS ${COMMON_CFLAGS} "-std=c++11 -fPIC")
ENDIF()

IF(DEFINED MSVC)
	# MSVC parallel builds by default
	SET(COMMON_CFLAGS ${COMMON_CFLAGS} /MP)
ENDIF()

# If doing a debug build, set the appropriate compiler defines
IF("${CMAKE_BUILD_TYPE}" MATCHES "Debug")
	add_definitions(-D_DEBUG)
ENDIF()

	set(configappend "")
	if (MSVC)
		if (CMAKE_CL_64)
			set(configappend "_x64")
		else()
			set(configappend "_x86")
		endif()
		add_definitions(-DCONF="$(Configuration)${configappend}")
		set(CONF CONF)
	endif()
	if (MINGW)
		add_definitions(-DCONF="${CMAKE_BUILD_TYPE}")
		set(CONF "\"${CMAKE_BUILD_TYPE}\"")
	endif()

endmacro(addBaseProject)

