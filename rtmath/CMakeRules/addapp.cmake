# CMake script for the very repetitive structure of 
# rtmath-apps CMakeLists.txt files.

# Take the variable {appname}, and link libraries,
# set properties and create an INSTALL target
macro(addapp appname)
	set(${appname}_LIBRARIES "")
	set(${appname}_INCLUDE_DIRS "")

	add_package_dependency(${appname} DEPENDS_ON rtmathDummy)
	target_link_libraries (${appname} ${${appname}_LIBRARIES})
IF(DEFINED COMMON_LIBS )
	target_link_libraries (${appname} ${COMMON_LIBS})
ENDIF()
#IF(DEFINED rtmath_INCLUDE_DIRS)
	# Other apps use this function. Only include rtmath if desired.
	target_link_libraries (${appname} rtmath)
#ENDIF()
	include_directories(${CMAKE_CURRENT_BINARY_DIR})
	include_directories(${${appname}_INCLUDE_DIRS})


	IF(DEFINED COMMON_CFLAGS) 
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
