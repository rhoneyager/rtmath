#include <string>
#include <iostream>
#include "../matrixop.h"

//#define BOOST_TEST_MODULE matrixop
//#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_matrixop);

// Check set/get alignment
BOOST_AUTO_TEST_CASE(matrixop_set) {
	rtmath::matrixop a(2,4,4);
	a.set(1.0,2,0,0);
	a.set(4.0,2,3,2);
	BOOST_CHECK_EQUAL(a.get(2,0,0),1.0);
	BOOST_CHECK_EQUAL(a.get(2,3,2),4.0);
}

// Check memory sizing
BOOST_AUTO_TEST_CASE(matrixop_sizing) {
	rtmath::matrixop a(4,5,4,3,2);
	BOOST_CHECK_EQUAL(a.maxSize(),120);
}

// Check read and write double array
BOOST_AUTO_TEST_CASE(matrixop_arrays) {
	rtmath::matrixop a(2,4,4);
	const size_t n = 16;
	const double vals[n] = {
		1,2,3,4,5,6,7,8,
		9,10,11,12,13,14,15,16 };
	a.fromDoubleArray(vals);
	BOOST_CHECK_EQUAL(a.get(2,0,0),1.0);
	BOOST_CHECK_EQUAL(a.get(2,1,0),5.0);
	BOOST_CHECK_EQUAL(a.get(2,2,0),9.0);
	BOOST_CHECK_EQUAL(a.get(2,0,1),2.0);
	BOOST_CHECK_EQUAL(a.get(2,0,2),3.0);
	BOOST_CHECK_EQUAL(a.get(2,3,3),16.0);
	double cvals[n];
	a.toDoubleArray(cvals);
	for (size_t i=0;i<n;i++)
		BOOST_CHECK_EQUAL(vals[i],cvals[i]);
	a.clear();
	a = vals;
	a.toDoubleArray(cvals);
	for (size_t i=0;i<n;i++)
		BOOST_CHECK_EQUAL(vals[i],cvals[i]);
}

// Check equality / inequality
BOOST_AUTO_TEST_CASE(matrixop_equality) {
	rtmath::matrixop a(2,4,4), b(2,4,4);
	const size_t n = 16;
	const double vals[n] = {
		1,2,3,4,5,6,7,8,
		9,10,11,12,13,14,15,16 };
	a.fromDoubleArray(vals);
	b.fromDoubleArray(vals);
	bool res = false;
	if (a == b) res = true;
	BOOST_CHECK_EQUAL(res,true);
	b.set(3.0,2,0,0);
	(a == b)? res = true : res = false;
	BOOST_CHECK_EQUAL(res,false);
	(a != b)? res = true : res = false;
	BOOST_CHECK_EQUAL(res,true);
}

// Check matrix addition and subtraction
BOOST_AUTO_TEST_CASE(matrixop_adddition_subtraction) {
	rtmath::matrixop a(2,4,4), b(2,4,4), c(2,4,4), d(2,4,4);
	const size_t n = 16;
	const double ai[n] = {
		1,2,3,4,5,6,7,8,
		9,10,11,12,13,14,15,16 };
	const double bi[n] = {
		0.5,1.5,2.5,3.5,4.5,5.5,6.5,7.5,8.5,9.5,
		9.5,0.4,0.3,6.2,3.1,2.6 };
	double cot[n], dot[n];
	a.fromDoubleArray(ai);
	b.fromDoubleArray(bi);
	c = a + b;
	d = a - b;
	c.toDoubleArray(cot);
	d.toDoubleArray(dot);
	for (size_t i=0;i<n;i++)
	{
		BOOST_CHECK_EQUAL(cot[i],ai[i]+bi[i]);
		BOOST_CHECK_EQUAL(dot[i],ai[i]-bi[i]);
	}
}

// Check constant multiplication
BOOST_AUTO_TEST_CASE(matrixop_const_mult) {
	rtmath::matrixop a(2,4,4);
	const size_t n=16;
	const double ai[n] = {
		1,2,3,4,5,6,7,8,
		9,10,11,12,13,14,15,16 };
	a.fromDoubleArray(ai);
	rtmath::matrixop b(2,4,4);
	b = a * 2.0;
	double bi[n];
	b.toDoubleArray(bi);
	for (size_t i=0;i<n;i++)
		BOOST_CHECK_EQUAL(ai[i]*2.0,bi[i]);
}

// Check matrix multiplication
BOOST_AUTO_TEST_CASE(matrixop_mat_mult) {
	rtmath::matrixop a(2,2,2), b(2,2,2), res(2,2,2);
	const size_t n=4;
	const double ai[n] = { 1,2,3,4 };
	const double bi[n] = { 2,3,4,5 };
	const double er[n] = { 10,13,22,29 };
	double ci[n];
	a.fromDoubleArray(ai);
	b.fromDoubleArray(bi);
	res = a * b;
	res.toDoubleArray(ci);
	for (size_t i=0;i<n;i++)
		BOOST_CHECK_EQUAL(ci[i],er[i]);
}

// Check matrix column set

// Check matrix row set

// Check matrix dimensionality

// Check matrix resizing

// Check matrix clearing

// Check matrix raised to power

// Check matrix cloning

// Check matrix construction methods

// Check matrix square status flag

// Check upperTriangular
BOOST_AUTO_TEST_CASE(matrixop_upperTriangular)
{
	using namespace std;
	rtmath::matrixop a(2,3,3), b(2,3,3), c(2,3,3);
	const double ai[] = { 1,2,3,4,5,6,7,9,8};
	const double bi[] = { 1,2,3,0,-3,-6,0,0,-3};
	a = ai;
//	a.print();
	b = bi;
//	b.print();
	c = a.upperTriangular();
//	c.print();
	//size_t d = a.dimensionality();
	bool res = false;
	if (b==c) res=true;

//	cout << "C " << c << " C";

	BOOST_CHECK_EQUAL(res,true);
//	BOOST_CHECK_EQUAL(b,c); // Unfortunately doesn't work
}

// Check lowerTriangular

// Check transpose
BOOST_AUTO_TEST_CASE(matrixop_transpose) {
	rtmath::matrixop a(2,4,4), b(2,4,4), c(2,4,4);
	const double ai[] = { 	1,2,3,4,
				5,6,7,8,
				9,10,11,12,
				13,14,15,16 };
	const double bi[] = { 	1,5,9,13,
				2,6,10,14,
				3,7,11,15,
				4,8,12,16 };
	a = ai;
	b = bi;
	c = a.transpose();
	bool res = false;
	if (b == c) res = true;
	BOOST_CHECK_EQUAL(res,true);
}


// Check determinant calculation
BOOST_AUTO_TEST_CASE(matrixop_det) {
	rtmath::matrixop a(2,3,3);
	const double ai[] = { 1,2,3,4,5,6,7,8,9 };
	a = ai;
	//a.print();
	double det = 1, res = 0;
	det = a.det();
	BOOST_CHECK_EQUAL(det,res);
	const double bi[] = {1,2,3,4,5,6,7,9,8};
	a = bi;
	det = a.det();
	res = 9;
	BOOST_CHECK_EQUAL(det,res);
}

// Check QR decomposition

// Check HouseholderUT

// Check upperHessenberg

// Check QRalgorithm

// Check minor calculation

// Check inverse calculation

// Check index / position interconversions

// Check diagonal construction

// Check identity construction

// Check file reads

// Check row swapping

// Check repivoting

// Check data allocation / freeing correct pointers

// Check row multiplication

// Check _getpos


BOOST_AUTO_TEST_SUITE_END();

