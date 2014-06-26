# Common include file that provides a program with subversion information

# Universal grep
macro(univ_grep findtxt filename outname)
	if (WIN32 AND NOT CYGWIN)
		EXECUTE_PROCESS(COMMAND "FindStr" "/B" ${findtxt} ${filename}
			WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
			OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${outname}"
			)
	else()
		EXECUTE_PROCESS(COMMAND "grep" ${findtxt} ${filename}
			WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
			OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${outname}"
			)
	endif()
endmacro()


EXECUTE_PROCESS(COMMAND "svn" "info" "."
	WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
	RESULT_VARIABLE res
	OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/SVN-INFO"
	ERROR_FILE "${CMAKE_CURRENT_BINARY_DIR}/SVN-ERROR"
	)
#message("${res} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}")

univ_grep("Revis" "SVN-INFO" "SVN-REV")
univ_grep("URL:\ ht" "SVN-INFO" "SVN-URL")
univ_grep("UUID" "SVN-INFO" "SVN-UUID")
univ_grep("Date" "SVN-INFO" "SVN-DATE")

file(READ "${CMAKE_CURRENT_BINARY_DIR}/SVN-REV" SVNREVISION
	OFFSET 10
	)
STRING(REGEX REPLACE "(\r?\n)+$" "" SVNREVISION "${SVNREVISION}")
if (NOT SVNREVISION)
	set (SVNREVISION 0)
endif()

file(READ "${CMAKE_CURRENT_BINARY_DIR}/SVN-URL" SVNURL
	OFFSET 5
	)
STRING(REGEX REPLACE "(\r?\n)+$" "" SVNURL "${SVNURL}")
if (NOT SVNURL)
	set (SVNURL "Unknown")
endif()

file(READ "${CMAKE_CURRENT_BINARY_DIR}/SVN-UUID" SVNUUID
	OFFSET 17
	)
STRING(REGEX REPLACE "(\r?\n)+$" "" SVNUUID "${SVNUUID}")
if (NOT SVNUUID)
	set (SVNUUID "Unknown")
endif()

file(READ "${CMAKE_CURRENT_BINARY_DIR}/SVN-DATE" SVNDATE
	OFFSET 19
	)
STRING(REGEX REPLACE "(\r?\n)+$" "" SVNDATE "${SVNDATE}")
if (NOT SVNDATE)
	set (SVNDATE "Unknown")
endif()

