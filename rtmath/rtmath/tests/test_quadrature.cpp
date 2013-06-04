
//#define _DEBUGS

#include <string>
#include <cmath>
#include <iostream>
#include <algorithm>
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"

//#define BOOST_TEST_MODULE matrixop
//#define BOOST_TEST_DYN_LINK

using namespace rtmath;

auto testeval = [](double val)
{
	return exp(val);
};

auto testevalb = [](double val)
{
	double res = 0.0;
	res = sin(val) * cos(val);
	return res;
};

BOOST_AUTO_TEST_SUITE(test_quadrature);


BOOST_AUTO_TEST_CASE(quadrature_eval_precomputed) {

	double res = 0;
	const unsigned int numExp = 7;
	const double expected[numExp] = {
		0,
		51.131072930036638,
		51.865715263807758,
		51.879725697449501,
		51.879867777028771,
		51.879867614428633,
		51.879868371736066
	};
	const double a = 1, b = 4;

	// Throws for i=1 from quadrature code
	for (unsigned int i=2; i<=numExp; i++)
	{
		res = rtmath::quadrature::quad_eval_leg(a,b,i, testeval);
		// Calculate percent deviation from actual values
		//double dev = (res - expected[i-1])/expected[i-1];
		//BOOST_CHECK( dev < 0.01 );
		//BOOST_CHECK( dev > -0.01 );
		BOOST_CHECK_CLOSE(res,expected[i-1],0.01);
	}

}

/*
BOOST_AUTO_TEST_CASE(quadrature_check_pts_gl) {
	std::set<rtmath::quadrature::ptWeight> pw;

	std::cout << "Checking Gauss-Legendre quadrature points\n";
	for (size_t i=2; i< 17; i++)
	{
		std::cout << "i = " << i;
		rtmath::quadrature::getQuadPtsLeg(i, pw);
		for (auto it = pw.begin(); it != pw.end(); ++it)
		{
			std::cout << "\t" << it->first << "\t" << it->second << "\n";
		}
	}

	BOOST_CHECK_CLOSE(0.001,0.01,0.01);
}
*/


BOOST_AUTO_TEST_CASE(quadrature_eval_zerocalced)
{
	return; // TODO: implement test
	const unsigned int deg = 8;
	const double expected = 0;
	double res = 0;
	const double a = 1, b = 4;
	
	try {
	res = rtmath::quadrature::quad_eval_leg(a,b,deg, testevalb);
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		BOOST_FAIL("Test failed. Error shown above.");
	}
	BOOST_CHECK_CLOSE(res,expected,0.01);
}

BOOST_AUTO_TEST_SUITE_END();

