macro(addBoostUniform )
	#set(Boost_DEBUG ON)
#set(BOOST_ALL_DYN_LINK ON)
set (BOOST_STATIC_LINK OFF)
#option (BOOST_STATIC_LINK "Mostly link static boost libraries" OFF)
if (NOT BOOST_STATIC_LINK)
	set(Boost_USE_STATIC_RUNTIME OFF)
	set(Boost_USE_STATIC_LIBS OFF)
endif()
if (WIN32 AND NOT CYGWIN)
	#option ( AUTOLINK_BOOST
	#	"Automatically link Boost" ON)
		set(WINBOOST_AUTOLINK ON) #${AUTOLINK_BOOST})
else()
	set(WINBOOST_AUTOLINK OFF)
endif()
#message("argv ${ARGV} ${args}")
find_package(Boost COMPONENTS ${ARGV} REQUIRED)
if (NOT WINBOOST_AUTOLINK)
	if (Boost_LIBRARIES)
		set(liblist ${liblist} ${Boost_LIBRARIES})
	endif()
	add_definitions(-DBOOST_ALL_NO_LIB)
	add_definitions(-DBOOST_LOG_DYN_LINK)
else()
	if (NOT BOOST_STATIC_LINK)
		add_definitions(-DBOOST_ALL_DYN_LINK)
		add_definitions(-DBOOST_LOG_DYN_LINK)
	endif()
endif()

include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})
#message("boost ${Boost_LIBRARY_DIR}")
link_directories(${Boost_LIBRARY_DIR})


# Also install / pack the relevant boost libraries into the install tree
# Do extention swapping
#if(WIN32)
#	set(dllext ".dll")
#	set(libext "[.]lib")
#else()
#	set(dllext ".so")
#	set(libext "[.]a")
#endif()


endmacro(addBoostUniform args)

