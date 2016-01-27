if (WIN32)
set (TIMESTAMP_PROVIDER /t http://timestamp.verisign.com/scripts/timestamp.dll
	CACHE STRING "Specifies the site used for timestamping signed code")
# /t http://timestamp.verisign.com/scripts/timestamp.dll
# /tr http://www.startssl.com/timestamp
set (SIGN_BINARIES TRUE CACHE BOOL "Sign the binaries")
else()
	set (SIGN_BINARIES FALSE)
endif()

macro(signing appname )
	if (WIN32)
		add_custom_command(TARGET ${appname} POST_BUILD #${appname}_Signed
			#COMMAND mt.exe -manifest ${subset} -outputresource:"$<TARGET_FILE:${appname}>"\;\#1
			COMMAND signtool.exe sign ${TIMESTAMP_PROVIDER} $<TARGET_FILE:${appname}>
			#DEPENDS ${appname}
			#COMMENT "Appending manifest in app ${appname} with ${subset} and signing."
			#WORKING_DIRECTORY "$<TARGET_FILE_DIR:${appname}>"
			)
	endif()
endmacro(signing appname )

macro(delayedsigning appname)
#	if (SIGN_BINARIES)
#		set (dsfiles ${dsfiles} ${appname} CACHE INTERNAL "DSFILES")
		#		set (dsfiles ${dsfiles} $<TARGET_FILE:${appname}> )
		#	endif()
endmacro()

