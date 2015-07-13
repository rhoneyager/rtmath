# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.
include (common)
include (commonRtmathApps)

find_package(CGAL)




if (CGAL_FOUND)
  include( ${CGAL_USE_FILE} )
  include( CGAL_CreateSingleSourceCGALProgram )

  include_directories (BEFORE include)

  create_single_source_cgal_program( "test1.cpp" )
  addapp(test1 CGAL_testing)
 
else(CGAL_FOUND)

  message(STATUS "NOTICE: This directory requires CGAL library, and will not be compiled.")

endif(CGAL_FOUND)
