include (common)

add_executable (mtplot
	main.cpp
	ui_frmMain.cpp
	frmMain.cpp
	)

addapp(mtplot)
#addbasicapp(mttest)
#if(LOAD_PACKAGES)
#	add_package_dependency(mttest DEPENDS_ON Boost)
#	target_link_libraries(mttest
#		boost_filesystem boost_serialization boost_iostreams boost_program_options boost_unit_test_framework boost_system)
#endif()
	target_link_libraries (mtplot ${mttest_LIBRARIES})
TARGET_LINK_LIBRARIES( mtplot tmatrix-cpp tmatrix-fortran rtmath rycurses netcdf)

