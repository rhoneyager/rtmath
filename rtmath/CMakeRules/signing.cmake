macro(signing appname subset)
	add_custom_command(TARGET ${appname} #${appname}_Signed
		COMMAND mt.exe -manifest ${subset} -outputresource:"$<TARGET_FILE:${appname}>"\;\#1
		COMMAND signtool.exe sign /t http://timestamp.verisign.com/scripts/timestamp.dll $<TARGET_FILE:${appname}>
		DEPENDS ${appname}
		COMMENT "Appending manifest in app ${appname} with ${subset} and signing."
		WORKING_DIR $<TARGET_FILE_DIR:${appname}>
		)
endmacro(signing appname deps)

