// This program produces a bunch of simple plots
// It uses the boost unit test framework to help select the type of plot to be made.
// Why this? Because it has built-in parsing and because I don't have to code so many 
// functions and headers.
// It's also useful for fast prototyping

#include <string>
#include <iostream>
#include <exception>

#define BOOST_TEST_MODULE raw-plotdata
#define BOOST_TEST_NO_MAIN
//#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK

//#include "globals.h"

#include <boost/test/unit_test.hpp>

//BOOST_GLOBAL_FIXTURE( globals );

#include "../rtmath/rtmath/error/debug.h"
#include "../rtmath/rtmath/ROOTlink.h"



int BOOST_TEST_CALL_DECL
main( int argc, char* argv[] )
{
	try {
		rtmath::debug::appEntry(argc,argv);
		rtmath::debug::debug_preamble();

		return ::boost::unit_test::unit_test_main( &init_unit_test, argc, argv );
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		exit(2);
	}
}
