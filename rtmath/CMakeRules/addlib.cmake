# CMake script to add a rtmath library

macro(addlib libname libshared)
	#if (NOT ${appname}_INCLUDE_DIRS)
	#	set(${appname}_LIBRARIES "")
	#	set(${appname}_INCLUDE_DIRS "")
	#endif()
	IF(DEFINED COMMON_CFLAGS )
		set_target_properties(${libname} PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	endif()
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES RELEASE_POSTFIX _Release )
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel )
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo )
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES DEBUG_POSTFIX _Debug )
	set_target_properties( ${libname} PROPERTIES FOLDER "Libs")

	
	# This is for determining the build type (esp. used in registry code)
	#target_compile_definitions(${libname} PRIVATE
	#	BUILDTYPE="${CMAKE_BUILD_TYPE}")
	target_compile_definitions(${libname} PRIVATE BUILDTYPE="$<CONFIGURATION>")
	# These two are for symbol export
	target_compile_definitions(${libname} PRIVATE EXPORTING_${libname})
	target_compile_definitions(${libname} PUBLIC SHARED_${libname}=$<STREQUAL:${libshared},SHARED>)
	#set (${libname}-libs rtmath_core ${rtmath_core-libs})
	#target_link_libraries(${libname} ${libname}-libs)

endmacro(addlib libname)


