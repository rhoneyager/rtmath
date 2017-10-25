macro(manifest_append appname subset)
	add_custom_command(TARGET ${appname} #${appname}_Manifest_Append
		COMMAND mt.exe -manifest ${subset} -outputresource:"$<TARGET_FILE:${appname}>"\;\#1
		# -inputresource:"$<TARGET_FILE:${appname}>"\;\#1 
		DEPENDS ${appname}
		COMMENT "Appending manifest in app ${appname} with ${subset}"
		)
endmacro(manifest_append appname subset)

