#include <string>
#include <iostream>
#include <memory>
#define BOOST_TEST_DYN_LINK
#include "../rtmath/da/damatrix.h"
#include "../rtmath/da/daDiagonalMatrix.h"
#include "../rtmath/da/daInitLayer.h"
#include "../rtmath/da/daLayer.h"
#include "../rtmath/da/damatrix_override.h"
#include "../rtmath/da/damatrix_quad.h"
#include "../rtmath/da/daPf.h"

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_damatrix);

// These tests are designed to test the functionality of atmos and its associated classes.
// It will lead a sample atmosphere and perform basic optical depth calculations.
// To accomplish this, however, the functionality of the absorber derived classes and of 
// the atmoslayer class must also be verified.

using namespace std;
using namespace rtmath;

// Construct basic damatrix from a matrixop
BOOST_AUTO_TEST_CASE(damatrix_daMatrixop_equality)
{
	matrixop a = matrixop::diagonal(1.0,2,4,4);
	std::shared_ptr<const matrixop> b(new matrixop(a));
	std::shared_ptr<damatrix> daA (new daMatrixop(a));
	std::shared_ptr<damatrix> daB (new daMatrixop(b));
	// Check that both damatrices are equal
	rtmath::mapid v(1.0,0.0,1.0,0.0);
	auto resA = daA->eval(v);
	auto resB = daB->eval(v);
	BOOST_CHECK(*resA == *resB);
	BOOST_CHECK(resA->get(2,0,0) == 1.0);
	BOOST_CHECK(resB->get(2,0,1) == 0.0);
}

// Try to load a phase function into a damatrix-derived class

// Test pf interpolation

// Test damatrix from mie scattering matrix provider

// Test Rayleigh scattering provider

// Test damatrix addition, normal multiplication, power and mult w/quad int
BOOST_AUTO_TEST_CASE(damatrix_simplemath)
{
	try {
		matrixop a = matrixop::diagonal(1.0,2,4,4);
		const double sb[] = {1.0,2,3,4,5,6,7,8,7,6,5,4,3,2,1,0};
		const double sres[] = {2,2,3,4,5,7,7,8,7,6,6,4,3,2,1,1};
		matrixop b(2,4,4);
		b.fromDoubleArray(sb);
		shared_ptr<damatrix> dA(new daMatrixop(a));
		shared_ptr<damatrix> dB(new daMatrixop(b));
		shared_ptr<damatrix> add( new damatrix(dA,dB,ADD));
		auto regmult = damatrix::op(dB,dB,MULTNORMAL);
		//auto pw = damatrix::op(dA,dB,POW);
		auto pw = damatrix::pow(dB,2);
		auto intmult = damatrix::op(dA,dB,MULT);
		auto inv = damatrix::op(dA,nullptr,INV);

		mapid mapjunk(1,0,1,0);

		matrixop addres(2,4,4);
		addres.fromDoubleArray(sres);


		// Do evaluations
		auto mA = dA->eval(mapjunk);
		auto mB = dB->eval(mapjunk);
		auto mAdd = add->eval(mapjunk);
		auto mRegmult = regmult->eval(mapjunk);
		auto mPw = pw->eval(mapjunk);
		auto mIntmult = intmult->eval(mapjunk);
		auto mInv = inv->eval(mapjunk);

		/*
		cout << "mA\n" << *mA << endl;
		cout << "mB\n" << *mB << endl;
		cout << "mAdd\n" << *mAdd << endl;
		cout << "mRegmult\n" << *mRegmult << endl;
		cout << "mPw\n" << *mPw << endl;
		cout << "mIntmult\n" << *mIntmult << endl;
		cout << "mInv\n" << *mInv << endl;
		*/

		BOOST_CHECK( *mRegmult == *mPw ); // Should produce same output
		BOOST_CHECK( *mAdd == addres ); // Check addition
		BOOST_CHECK( *mInv == a); // Inverse should be easy. b is singular.

		const double spow[] = { 44,40,36,32,108,104,100,96,84,88,92,96,20,24,28,32 };
		matrixop pres(2,4,4);
		pres.fromDoubleArray(spow);
		BOOST_CHECK( *mPw == pres); // Check squaring of matrix
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
	}
}

// Test damatrix multiplication and integration by quadrature


// Test thin layer generation

// Test basic doubling

// Test recursion of ~ 5 layers, and check appropriate cacheing

BOOST_AUTO_TEST_SUITE_END();

