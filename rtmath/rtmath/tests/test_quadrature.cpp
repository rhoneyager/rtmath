
//#define _DEBUGS

#include <string>
#include <cmath>
#include <iostream>
#include <boost/test/unit_test.hpp>

#include "../error.h"
#include "../quadrature.h"

//#define BOOST_TEST_MODULE matrixop
//#define BOOST_TEST_DYN_LINK

using namespace rtmath;
class testeval : public rtmath::evalfunction
{
	public:
	testeval() {}
	virtual ~testeval() {}
	virtual double eval(double val) const
	{
		// Function evaluates to e^x 
		return exp(val);
		/*
		double r = 1.0;
		double denom = 1.0;
		double res = 0.0;
		for (int i=1;i<=7;i++)
		{
			r *= val;
			denom *= (double) i;
			res += r / denom;
		}
		//std::cout << res << std::endl;
		return res;
		*/
	}

	virtual double operator() (double val) const { return eval(val); }
};

class testevalb : public testeval
{
	virtual double eval(double val) const
	{
		double res = 0.0;
		res = sin(val) * cos(val);
		return res;
	}
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

	testeval ev;

	// Throws for i=1 from quadrature code
	for (unsigned int i=2; i<=numExp; i++)
	{
		res = rtmath::quadrature::quad_eval_leg(a,b,i, &ev);
		// Calculate percent deviation from actual values
		//double dev = (res - expected[i-1])/expected[i-1];
		//BOOST_CHECK( dev < 0.01 );
		//BOOST_CHECK( dev > -0.01 );
		BOOST_CHECK_CLOSE(res,expected[i-1],0.01);
	}

}


BOOST_AUTO_TEST_CASE(quadrature_eval_zerocalced)
{
	const unsigned int deg = 8;
	const double expected = 0;
	double res = 0;
	const double a = 1, b = 4;
	testevalb ev;
	try {
	res = rtmath::quadrature::quad_eval_leg(a,b,deg, &ev);
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		BOOST_FAIL("Test failed. Error shown above.");
	}
	BOOST_CHECK_CLOSE(res,expected,0.01);
}

BOOST_AUTO_TEST_SUITE_END();

