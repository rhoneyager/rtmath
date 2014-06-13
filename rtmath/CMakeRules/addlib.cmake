# CMake script to add a rtmath library

macro(addlib libname libshared )
	#if (NOT ${appname}_INCLUDE_DIRS)
	#	set(${appname}_LIBRARIES "")
	#	set(${appname}_INCLUDE_DIRS "")
	#endif()
	if ("" STREQUAL "${ARGV2}")
		set(headername ${libname})
	else()
		set(headername ${ARGV2})
	endif()
	#message("${headername} ${ARGV2} ${libname} ${ARGC} ${ARGV}")


	#	IF(DEFINED COMMON_CFLAGS )
	#	set_target_properties(${libname} PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	#endif()
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES RELEASE_POSTFIX _Release${configappend} )
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES MINSIZEREL_POSTFIX _MinSizeRel${configappend} )
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES RELWITHDEBINFO_POSTFIX _RelWithDebInfo${configappend} )
	SET_TARGET_PROPERTIES( ${libname} PROPERTIES DEBUG_POSTFIX _Debug${configappend} )
	set_target_properties( ${libname} PROPERTIES FOLDER "Libs")

	
	# This is for determining the build type (esp. used in registry code)
	#target_compile_definitions(${libname} PRIVATE
	#	BUILDTYPE="${CMAKE_BUILD_TYPE}")
	target_compile_definitions(${libname} PRIVATE BUILDCONF="${CMAKE_BUILD_TYPE}")
	target_compile_definitions(${libname} PRIVATE BUILDTYPE=BUILDTYPE_$<CONFIGURATION>)
	# These two are for symbol export
	target_compile_definitions(${libname} PRIVATE EXPORTING_${headername})
	target_compile_definitions(${libname} PUBLIC SHARED_${headername}=$<STREQUAL:${libshared},SHARED>)
	#set (${libname}-libs rtmath_core ${rtmath_core-libs})
	#target_link_libraries(${libname} ${libname}-libs)

endmacro(addlib libname headername)

macro(storebin objname)
set_target_properties( ${objname}
    PROPERTIES
    #  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    # LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_BINARY_DIR}/MinSizeRel"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_BINARY_DIR}/RelWithDebInfo"
)

endmacro(storebin objname)

