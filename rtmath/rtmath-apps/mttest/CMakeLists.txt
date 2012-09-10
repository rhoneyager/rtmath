include (common)

add_executable (mttest
	main.cpp
	ui_frmMain.cpp
	frmMain.cpp
	)

addapp(mttest)
#addbasicapp(mttest)
#if(LOAD_PACKAGES)
#	add_package_dependency(mttest DEPENDS_ON Boost)
#	target_link_libraries(mttest
#		boost_filesystem boost_serialization boost_iostreams boost_program_options boost_unit_test_framework boost_system)
#endif()
	target_link_libraries (mttest ${mttest_LIBRARIES})
TARGET_LINK_LIBRARIES( mttest tmatrix-cpp tmatrix-fortran rtmath rycurses netcdf)

