
//#define _DEBUGS

#include <string>
#include <cmath>
#include <iostream>
#include <algorithm>
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include "../rtmath/mie/mie-piNCalc.h"
#include "../rtmath/mie/mie-tauNCalc.h"
#include "../rtmath/mie/mie-wnCalc.h"
#include "../rtmath/mie/mie-AnCalc.h"
#include "../rtmath/error/error.h"

//#define BOOST_TEST_MODULE matrixop
//#define BOOST_TEST_DYN_LINK

using namespace rtmath;

BOOST_AUTO_TEST_SUITE(test_mie_tn_wn);

BOOST_AUTO_TEST_CASE(mie_pin)
{
	using namespace rtmath::mie;
	using namespace rtmath::debug;
	try
	{
		piNCalc t0(0);
		piNCalc t1(1);
		size_t numChecks = 5;
		const double check_0[] = { 0, 0, 0, 0, 0
		};
		const double check_1[] = { 0, 0, 0, 0, 0
		};

		std::cout << "pin\n";
		for (size_t i=0; i<numChecks; i++)
		{
			double res = t0.calc(i);
			std::cout << res << "\t";
			//BOOST_CHECK_CLOSE(res,check_0[i],0.0001);
			res = t1.calc(i);
			std::cout << res << "\n";
			//BOOST_CHECK_CLOSE(res,check_1[i],0.0001);
		}
	}
	catch (std::exception &)
	{
	}
}

BOOST_AUTO_TEST_CASE(mie_tn)
{
	using namespace rtmath::mie;
	using namespace rtmath::debug;
	try
	{
		tauNCalc t0(0);
		tauNCalc t1(1);
		size_t numChecks = 5;
		const double check_0[] = { 0, 0, 0, 0, 0
		};
		const double check_1[] = { 0, 0, 0, 0, 0
		};

		std::cout << "tn\n";
		for (size_t i=1; i<numChecks; i++)
		{
			double res = t0.calc(i);
			std::cout << res << "\t";
			//BOOST_CHECK_CLOSE(res,check_0[i],0.0001);
			res = t1.calc(i);
			std::cout << res << "\n";
			//BOOST_CHECK_CLOSE(res,check_1[i],0.0001);
		}
	}
	catch (std::exception &)
	{
	}
}

BOOST_AUTO_TEST_CASE(mie_wn)
{
	using namespace rtmath::mie;
	using namespace rtmath::debug;
	try
	{
		wnCalc w0(1);
		wnCalc w1(2);
		size_t numChecks = 5;
		std::vector<std::complex<double> > check_0(5), check_1(5);

		std::cout << "wn\n";
		for (size_t i=0; i<numChecks; i++)
		{
			std::complex<double> res = w0.calc(i);
			//BOOST_CHECK_CLOSE(res.real(),check_0[i].real(),0.0001);
			//BOOST_CHECK_CLOSE(res.imag(),check_0[i].imag(),0.0001);
			std::cout << res << "\t";
			res = w1.calc(i);
			//BOOST_CHECK_CLOSE(res.real(),check_1[i].real(),0.0001);
			//BOOST_CHECK_CLOSE(res.imag(),check_1[i].imag(),0.0001);
			std::cout << res << "\n";
		}
	}
	catch (std::exception &)
	{
	}
}

BOOST_AUTO_TEST_CASE(mie_An)
{
	using namespace rtmath::mie;
	using namespace rtmath::debug;
	try
	{
		AnCalc w0(1., std::complex<double>(1.33,0));
		AnCalc w1(2., std::complex<double>(1.5,0.01));
		size_t numChecks = 5;
		std::vector<std::complex<double> > check_0(5), check_1(5);

		std::cout << "An\n";
		for (size_t i=0; i<numChecks; i++)
		{
			std::complex<double> res = w0.calc(i);
			//BOOST_CHECK_CLOSE(res.real(),check_0[i].real(),0.0001);
			//BOOST_CHECK_CLOSE(res.imag(),check_0[i].imag(),0.0001);
			std::cout << res << "\t";
			res = w1.calc(i);
			//BOOST_CHECK_CLOSE(res.real(),check_1[i].real(),0.0001);
			//BOOST_CHECK_CLOSE(res.imag(),check_1[i].imag(),0.0001);
			std::cout << res << "\n";
		}
	}
	catch (std::exception &)
	{
	}
}

BOOST_AUTO_TEST_SUITE_END();

