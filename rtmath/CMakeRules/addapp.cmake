# CMake script for the very repetitive structure of 
# rtmath-apps CMakeLists.txt files.

# Take the variable {appname}, and link libraries,
# set properties and create an INSTALL target
macro(addapp appname)
#	target_link_libraries (${appname} rtmath ${COMMON_LIBS})

#	set(${appname}_INCLUDE_DIRS ${rtmath_INCLUDES})
#	get_target_property(${appname}_INCLUDE_DIRS ${appname} INCLUDE_DIRECTORIES)
	set(${appname}_LIBRARIES "")
	set(${appname}_INCLUDE_DIRS "")
#	set(${appname}_LIBRARIES ${rtmath_LIBRARIES})

	add_package_dependency(${appname} DEPENDS_ON rtmathDummy)
#	message("${appname} --- ${${appname}_INCLUDE_DIRS}")
#	message("${rtmath_INCLUDES}")
	target_link_libraries (${appname} ${${appname}_LIBRARIES})
	target_link_libraries (${appname} ${COMMON_LIBS} rtmath)
	include_directories("${CMAKE_CURRENT_BINARY_DIR}")
	include_directories(${${appname}_INCLUDE_DIRS})

#	message("${COMMON_LIBS}")
#	message("${${appname}_INCLUDE_DIRS}")

	IF(DEFINED COMMON_CFLAGS )
		set_target_properties(${appname} PROPERTIES COMPILE_FLAGS ${COMMON_CFLAGS})
	ENDIF()

	INSTALL(TARGETS ${appname} RUNTIME DESTINATION bin COMPONENT Applications)
	set_target_properties(${appname} PROPERTIES FOLDER "Apps")
endmacro(addapp appname)

macro(add_header_files srcs)
  if( hds )
    set_source_files_properties(${hds} PROPERTIES HEADER_FILE_ONLY ON)
    list(APPEND ${srcs} ${hds})
  endif()
endmacro(add_header_files srcs)
