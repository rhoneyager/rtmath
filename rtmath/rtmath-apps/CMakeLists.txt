# Build the apps

#add_subdirectory (absorb-slab)
#add_subdirectory (arm-index-files)
#add_subdirectory (atmosplot)
add_subdirectory (combo)
add_subdirectory (conv-units)
#add_subdirectory (da)
#add_subdirectory (db-catalog-ddscat)
#add_subdirectory (ddparse)
#add_subdirectory (ddscat)
add_subdirectory (ddscat-data)
add_subdirectory (ddscat-ddpar)
#add_subdirectory (ddscat-genparams)
add_subdirectory (expand-shorthand)
#add_subdirectory (garrett)
add_subdirectory (getm)
#add_subdirectory (roottest)
add_subdirectory (rtmath-version)
add_subdirectory (sector)
add_subdirectory (shape)
add_subdirectory (shape-basicdata)
#add_subdirectory (transmittance)
#add_subdirectory (uuid)

# Control option for creating qt apps
option ( MAKE_GUIS
	"Make Qt GUI programs?" OFF)

if (MAKE_GUIS)
	add_subdirectory (ddscat-genparams-qt)
	if (TMATRIX_FOUND)
		add_subdirectory (tmatrix-convert-from-ddscat)
		add_subdirectory (mie)
	endif()
endif()

if (RYCURSES_FOUND)
	option ( MAKE_NCURSES
		"Make ncurses programs?" ON)
	if (MAKE_NCURSES)
		if (TMATRIX_FOUND)
#			add_subdirectory (mtplot)
			add_subdirectory (mttest)
		endif()
	endif()
endif()

