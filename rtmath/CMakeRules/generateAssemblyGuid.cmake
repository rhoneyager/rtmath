macro(generateAssemblyGuid assemblyName config type output)

	# {77B702E8-5E6F-42AD-84ED-D31E691B6E2F}
	# {E1DFC1EE-7502-4E86-8EF4-8C8121A8757D}
	# {E9FFCBCE-5F4E-4F29-BC65-B99F08F3C04F}
	# {779993BD-3F58-4E0B-B40A-5EC35BE1D612}
	set(SXSGUID_Ryan_Debug 77B702E8-)
	set(SXSGUID_Debug 5E6F-)
	set(SXSGUID_Release 7502-)
	set(SXSGUID_RelWithDebInfo 5F4E-)
	set(SXSGUID_MinSizeRel 3F58-)
	set(SXSGUID_x64 42AD-)
	set(SXSGUID_x32 4E86-)
	set(SXSGUID_vc100 B40A-)
	set(SXSGUID_vc110 84ED-)
	set(SXSGUID_vc120 8EF4-)
	set(SXSGUID_mingw BC65-)
	set(SXSGUID_policy D31E691B6E2F)
	set(SXSGUID_public 8C8121A8757D)

	set (resguid ${SXSGUID_${assemblyName}}${SXSGUID_${config}})
	if (MSVC)
		if(MSVC10)
			set(resguid ${resguid}${SXSGUID_vc100})
		elseif(MSVC11)
			set(resguid ${resguid}${SXSGUID_vc110})
		elseif(MSVC12)
			set(resguid ${resguid}${SXSGUID_vc120})
		endif()
	endif()
	if (MINGW)
		set(resguid ${resguid}${SXSGUID_mingw})	
	endif()

	if (CMAKE_CL_64)
		set(resguid ${resguid}${SXSGUID_x64})
	else()
		set(resguid ${resguid}${SXSGUID_x32})
	endif()

	set(resguid ${resguid}${SXSGUID_${type}})

	set(${output} ${resguid})
	unset(resguid)
endmacro(generateAssemblyGuid assemblyName config type)

