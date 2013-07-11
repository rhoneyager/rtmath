// This is the master file that defines the test framework
#include <string>
#include <iostream>
#include <exception>
#include <Ryan_Debug/debug.h>

#define BOOST_TEST_MODULE rtmath
#define BOOST_TEST_NO_MAIN
//#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK

#include "globals.h"

#include <boost/test/unit_test.hpp>
//#include <boost/test/included/unit_test.hpp>

BOOST_GLOBAL_FIXTURE( globals );

#include "../rtmath/error/debug.h"




int BOOST_TEST_CALL_DECL
main( int argc, char* argv[] )
{
	try {
		//rtmath::debug::appEntry(argc,argv);
		//rtmath::debug::debug_preamble();

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

