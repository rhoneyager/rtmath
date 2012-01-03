#include <string>
#include <iostream>
#include "../quadrature.h"

//#define BOOST_TEST_MODULE matrixop
//#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
using namespace rtmath;
class testeval : public rtmath::evalfunction
{
	public:
	testeval() {}
	virtual ~testeval() {}
	virtual double eval(double val) const
	{
		return 0;
	}

	virtual double operator() (double val) const { return eval(val); }
};

BOOST_AUTO_TEST_SUITE(test_quadrature);


BOOST_AUTO_TEST_CASE(quadrature_eval_precomputed) {

	double res = 0;
	const unsigned int numExp = 7;
	const double expected[numExp] = {
		0,0,0,0,0,0,0
	};
	const double a = 2, b = 5;

	//testeval eval();

	for (unsigned int i=1; i<=numExp; i++)
	{
		//res = rtmath::quadrature::evalfunction(a,b,i, &eval);
		BOOST_CHECK_EQUAL(res,expected[i-1]);
	}

}

/*
BOOST_AUTO_TEST_CASE(quadrature_eval_zerocalced)
{
}
*/
BOOST_AUTO_TEST_SUITE_END();

