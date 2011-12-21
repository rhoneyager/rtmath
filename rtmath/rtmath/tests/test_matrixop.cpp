#include <string>
#include "../matrixop.h"

#define BOOST_TEST_MODULE matrixop
#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

// Check set/get alignment
BOOST_AUTO_TEST_CASE(matrixop_set) {
	rtmath::matrixop a(2,4,4);
	a.set(1.0,2,0,0);
	a.set(4.0,2,3,2);
	BOOST_CHECK_EQUAL(a.get(2,0,0),1.0);
	BOOST_CHECK_EQUAL(a.get(2,3,2),4.0);
}

// Check memory sizing
//
// Check read from double array
//
// Check write to double array
//
// Check matrix addition
//
// Check matrix subtraction
//
// Check constant multiplication
//
// Check matrix multiplication
//
// Check matrix column set
//
// Check matrix row set
//
// Check matrix dimensionality
//
// Check matrix resizing
//
// Check matrix clearing
//
// Check matrix raised to power
//
// Check matrix cloning
//
// Check matrix construxtion methods
//
// Check matrix equality / inequality
//
// Check matrix square status flag
//
// Check determinant calculation
//
// Check upperTriangular
//
// Check lowerTriangular
//
// Check transpose
//
// Check QR decomposition
//
// Check HouseholderUT
//
// Check upperHessenberg
//
// Check QRalgorithm
//
// Check minor calculation
//
// Check inverse calculation
//
// Check index / position interconversions
//
// Check diagonal construction
//
// Check identity construction
//
// Check file reads
//
// Check row swapping
//
// Check repivoting
//
// Check data allocation / freeing correct pointers
//
// Check row multiplication
//
// Check _getpos


