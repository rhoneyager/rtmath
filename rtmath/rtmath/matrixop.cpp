#include "Stdafx.h"
#include "matrixop.h"
// Reduces tedium when finding minors
//#include <boost/foreach.hpp>
#include <cmath>

namespace rtmath {

matrixop::matrixop(const std::vector<unsigned int> &size)
{
	_dims = size;
	// When getting values, if not in map, return 0
	// So, initialization need not occur yet
}

matrixop::~matrixop(void)
{
}

matrixop::matrixop(const matrixop & rhs)
{
	// The matrixop copy constructor
	this->_vals = rhs._vals;
	this->_dims = rhs._dims;
}

matrixop::matrixop(unsigned int ndims, ...)
{
	va_list indices;
	va_start(indices, ndims);
	std::vector<unsigned int> ptr;
	unsigned int ival;
	for (unsigned int i=0; i<ndims; i++)
	{
		ival = va_arg(indices,unsigned int);
		ptr.push_back(ival);
	}
	va_end(indices);
	// NOTE: cannot call matrixop(vector) directly
	// So reduplicate the necessary code
	// TODO: fix this
	_dims = ptr;
}

void matrixop::resize(const std::vector<unsigned int> &size)
{
	clear();
	_dims = size;
}

void matrixop::resize(unsigned int ndims, ...)
{
	va_list indices;
	va_start(indices, ndims);
	std::vector<unsigned int> ptr;
	unsigned int ival;
	for (unsigned int i=0; i<ndims; i++)
	{
		ival = va_arg(indices,unsigned int);
		ptr.push_back(ival);
	}
	va_end(indices);
	// NOTE: cannot call matrixop(vector) directly
	// So reduplicate the necessary code
	// TODO: fix this
	resize(ptr);
}

matrixop* matrixop::clone() const
{
	return new matrixop(*this);
}

unsigned int matrixop::dimensionality() const
{
	return (unsigned int) _dims.size();
}

void matrixop::size(std::vector<unsigned int> &out) const
{
	out = _dims;
}

const std::vector<unsigned int> matrixop::size() const
{
	//TODO: check to see if this code works as intended
	//NOTE: declaring as static failed
	//static std::vector<unsigned int> temp = _dims;
	//return temp;
	return _dims;
}

matrixop matrixop::operator+ (const matrixop& rhs) const
{
	if (rhs.dimensionality() != this->dimensionality()) throw;
	if (rhs.size() != this->_dims) throw;

	matrixop temp(_dims);
	temp = rhs;
	
	using namespace std;
	// Use const_iterator since this function is const
	map<std::vector<unsigned int>, double>::const_iterator it;
	for ( it = _vals.begin(); it != _vals.end(); it++)
		temp.set(it->first, temp.get(it->first) + it->second);
	return temp;
}

matrixop matrixop::operator* (const matrixop& rhs) const
{
	// Define standard matrix multiplication
	// That is: do the bilinear mapping between two rank 0,1 or 2 tensors
	// If greater dimensionality, throw an error since it's unsupported
	if (rhs.dimensionality() > 2) throw;
	if (this->dimensionality() > 2) throw;

	// size()[0] is number of rows, size()[1] is # of columns
	// If matrices cannot produce a square matrix, throw
	//std::vector<unsigned int> testA = this->size(), testB = rhs.size();
	if (this->size()[1] != rhs.size()[0]) throw;

	// Create vector for size of result
	using namespace std;
	vector<unsigned int> ressize;
	ressize.push_back(this->size()[0]); // number of rows
	ressize.push_back(rhs.size()[1]); // number of columns
	matrixop res(ressize);

	// Actually calculate the resulting values
	// TODO: optimize this
	for (unsigned int i=0; i< res.size()[0]; i++)
	{
		for (unsigned int j=0; j< res.size()[1]; j++)
		{
			vector<unsigned int> pos(2,0), pa(2,0), pb(2,0);
			pos[0] = i;
			pos[1] = j;
			// Calculate res(i,j)
			double val = 0;
			for (unsigned int k=0; k<this->size()[1]; k++)
			{
				pa[0] = i;
				pa[1] = k;
				pb[0] = k;
				pb[1] = j;
				double nres = this->get(pa) * rhs.get(pb);
				val += nres;
			}
			res.set(pos,val);
		}
	}
	//TODO: fix so that it returns!
	return res;

}

matrixop matrixop::operator* (double rhs) const
{
	matrixop temp(_dims);
	using namespace std;
	// Use const_iterator since this function is const
	map<std::vector<unsigned int>, double>::const_iterator it;
	for ( it = _vals.begin(); it != _vals.end(); it++)
		temp.set(it->first, it->second * rhs);
	return temp;
}

bool matrixop::issquare() const
{
	if (dimensionality() != 2) return false;
	if (_dims[0] == _dims[1]) return true;
	return false;
}

matrixop matrixop::operator^ (unsigned int rhs) const
{
	// In future, allow for fast power (using linear algebra)
	// For now, do the slow recursive matrix multiplication
	if (issquare() == false) throw;
	// Do recursion
	if (rhs == 0) // Handle special case of matrix to zero power
	{
		matrixop res(_dims);
		for (unsigned int i=0; i< _dims[0]; i++)
		{
			std::vector<unsigned int> pos(_dims[0],i);
			res.set(pos,1.0);
		}
		return res;
	} else if (rhs == 1) // Return original matrix
	{
		return *this;
	} else { // Do recursive matrix multiplication
		matrixop res(_dims);
		res = *this;
		for (unsigned int i=2; i<rhs; i++)
			res = res * (*this);
		return res;
	}
	// Should not reach this point!
	throw;
}

matrixop matrixop::operator- (const matrixop& rhs) const
{
	// Use multiplication defined by the BASE class, not from derived!
	if (rhs.dimensionality() != this->dimensionality()) throw;
	if (rhs.size() != this->_dims) throw;

	using namespace std;
	matrixop tempr(_dims);
	tempr = rhs.matrixop::operator*(-1.0); // use the base class def!

	matrixop temp(_dims);
	temp = (*this).matrixop::operator+(tempr);

	return temp;
}

void matrixop::clear()
{
	_vals.clear();
}

void matrixop::set(const std::vector<unsigned int> &pos, double val)
{
	// Check for going beyond matrix bounds
	if (pos.size() != _dims.size()) throw;
	for (unsigned int i=0; i<pos.size(); i++)
	{
		if (pos[i] > _dims[i]) throw;
	}
	// Set the value
	_vals[pos] = val;
}

void matrixop::set(double val, unsigned int rank, ...)
{
	// Uses variadic array to get the index, as matrices can have multiple dimensions
	if (rank != this->dimensionality()) throw;
	va_list indices;
	va_start(indices, rank);
	std::vector<unsigned int> ptr;
	unsigned int ival;
	for (unsigned int i=0; i<rank; i++)
	{
		ival = va_arg(indices,unsigned int);
		ptr.push_back(ival);
	}
	va_end(indices);
	set(ptr,val);
}

double matrixop::get(unsigned int rank, ...) const
{
	// Uses variadic array to get the index, as matrices can have multiple dimensions
	if (rank != this->dimensionality()) throw;
	va_list indices;
	va_start(indices, rank);
	std::vector<unsigned int> ptr;
	unsigned int ival;
	for (unsigned int i=0; i<rank; i++)
	{
		ival = va_arg(indices,unsigned int);
		ptr.push_back(ival);
	}
	va_end(indices);
	return get(ptr);
}

double matrixop::get(const std::vector<unsigned int> &pos) const
{
	// Check matrix bounds
	if (pos.size() != _dims.size()) throw;
	for (unsigned int i=0; i<pos.size(); i++)
	{
		if (pos[i] > _dims[i]) throw;
	}
	// Get the value (if it exists)
	// Otherwise, return a zero
	if (_vals.count(pos) > 0) 
	{
			return _vals.at(pos);
	}
	return 0;
}

bool matrixop::operator!= (const matrixop& rhs) const
{
	return !(this->operator==(rhs));
}

bool matrixop::operator== (const matrixop& rhs) const
{
	// Simply check each value. If they are the same, return true.

	// First, check the size
	if (_dims != rhs.size()) return false;

	// Same size. Check individual values
	using namespace std;
	// Use const_iterator since this function is const
	map<std::vector<unsigned int>, double>::const_iterator it;
	for ( it = _vals.begin(); it != _vals.end(); it++)
	{
		if (it->second != rhs.get(it->first)) return false;
	}
	return true;
}

matrixop matrixop::minor(unsigned int rank, ...) const
{
	// Retrieve the variable parameters
	va_list indices;
	va_start(indices, rank);
	std::vector<unsigned int> ptr;
	unsigned int ival;
	for (unsigned int i=0; i<rank; i++)
	{
		ival = va_arg(indices,unsigned int);
		ptr.push_back(ival);
	}
	va_end(indices);
	return this->minor(ptr);
}

void matrixop::_push_back(unsigned int index, double val)
{
	// Used by minor to place a value in the appropriate location
	using namespace std;
	vector<unsigned int> pos;
	_getpos(index,pos);
	// Set the value
	set(pos,val);
}

void matrixop::_getpos(unsigned int index, std::vector<unsigned int> &pos) const
{
	using namespace std;
	if (dimensionality() == 0) throw;
	pos.clear();
	pos.resize(_dims.size());
	pos[_dims.size()-1] = index;

	// If > dimensionality, do modular addition
	// BOOST_REVERSE_FOREACH(unsigned int i,pos) // REVERSE_FOREACH seems unreliable
	for (int i=_dims.size()-1;i>=0;i--)
	{
		if (pos[i] >= _dims[i] && i > 0)
		{
			pos[i-1] += pos[i] / _dims[i];
			pos[i] = pos[i] % _dims[i];
		}
	}
}

matrixop matrixop::minor(const std::vector<unsigned int> &rc) const
{
	using namespace std;
	if (_dims.size() != 2) throw; // Allow only 2d matrices
	vector<unsigned int> msiz = _dims;
	for (unsigned int i=0;i<msiz.size();i++)
	{
		msiz[i] = msiz[i] - 1;
		if (msiz[i] == 0) throw;
	}
	matrixop res(msiz);
	// I have the empty resultant matrix
	// Now, to fill in the minors
	unsigned int rowX=rc[0],colX=rc[1];
	int rS = 0, cS = 0; // Row / column shift
	// Since this is used in more than just determinants, I cannot assume that
	// the matrix is square. Too bad.
	// Iterate over the rows and columns of the initial matrix
	for (unsigned int srow=0;srow<_dims[0];srow++)
	{
		if (srow == rowX) rS = 1; // Add the skip factor for future rows
		if (srow == rowX) continue; // Don't even bother here
		cS = 0;
		for (unsigned int scol=0;scol < _dims[1];scol++)
		{
			if (scol == colX) cS = 1;
			if (scol == colX) continue;
			// Do the actual value copying here
			double val = get(2,srow,scol);
			res.set(val,2,srow-rS,scol-cS); // Row and column skip factors shift the result!
		}
	}
	return res;
}

double matrixop::det() const
{
	using namespace std;
	// Find the determinant
	if (!issquare()) throw;
	if (dimensionality() == 1) return get(1,0);
	// Check that matrix is 2d, for now
	// TODO: extend this to n dims, if possible
	if (_dims.size() != 2) throw;

	// Handle the 1x1 case easily
	if (_dims[0] == 1 && _dims[1] == 1) return get(2,0,0);
	// 2x2 case should be handled as well
	if (_dims[0] == 2 && _dims[1] == 2) return ((get(2,0,0)*get(2,1,1))-(get(2,0,1)*get(2,1,0)));
	// Do rest by expansion of minors.
	// It's unfortunately slow and relies heaviny on recursion, but
	// I don't want to implement a whole linear algrbra system just yet
	// TODO: add in gaussian elimination code

	
	vector<unsigned int> it;
	it.push_back(0); // 0th row
	it.push_back(0); // 0th column
	// Retrieve the first row of the matrix, get minor, and calculate determinant of minor
	double res = 0.0;
	// Wow, this next line is obtuse
	for (it[1] = 0; it[1] < _dims[1]; it[1]++)
	{
            std::vector<unsigned int> pos;
            pos.push_back(it[0]); pos.push_back(it[1]);
            //throw;
		matrixop min = this->minor(pos);
		// () alternates negatives, get(it) is the value at (0,i) and min.det is the minor's det
		res += std::pow(-1.0, (double) it[1]) * min.det() * get(it);
	}
	return res;
}

matrixop matrixop::diagonal(const std::vector<unsigned int> &size, double val)
{
	matrixop res(size);
	if (size.size() != 2) throw;
	// Find min dimension
	unsigned int min = (size[0] < size[1])? size[0] : size[1];
	for (unsigned int i=0;i<min;i++)
	{
		res.set(val,2,i,i);
	}
	return res;
}

matrixop matrixop::diagonal(double val, unsigned int rank, ...)
{
	// Retrieve the variable parameters
	va_list indices;
	va_start(indices, rank);
	std::vector<unsigned int> ptr;
	unsigned int ival;
	for (unsigned int i=0; i<rank; i++)
	{
		ival = va_arg(indices,unsigned int);
		ptr.push_back(ival);
	}
	va_end(indices);
	return diagonal(ptr,val);
}

matrixop matrixop::inverse() const
{
	// This uses minors and a determinant to calculate the inverse
	double mdet = det();
	if (mdet == 0) throw;
	// Construct the matrix minors
	// Assumes 2d array from det. TODO: extend if possible
	matrixop res(this->_dims);
	for (unsigned int i=0;i<_dims[0];i++) // columns
	{
		for (unsigned int j=0;j<_dims[1];j++) // rows
		{
                    std::vector<unsigned int> pos; pos.push_back(j); pos.push_back(i);
			// Calculate the appropriate minor
                    //throw;
			matrixop min = this->minor(pos);
			double inv = std::pow(-1.0, (double) (i+j) ) * min.det() / mdet;
			res.set(inv,2,j,i);
		}
	}
	return res;
}

matrixop matrixop::identity(const std::vector<unsigned int> &size)
{
	return diagonal(size,1.0);
}

void matrixop::toDoubleArray(double *target)
{
	// Take the data and convert to an array of doubles
	// Use pointers to do this, and assume that target is pre-sized appropriately
	// Start with the dimensions of the matrix, and produce a mapping
	// Use pointers like target[1] = 0.0;

	using namespace std;
	// Use const_iterator
	map<vector<unsigned int>, double>::const_iterator it;
		unsigned int runningTotal = 1;
		vector<unsigned int> mfactorflip, mfactor;
		// Get the position in the array
		for (unsigned int i= (unsigned int) _dims.size()-1;i!=0;i--)
		{
			mfactorflip.push_back(_dims[i] * runningTotal);
			runningTotal *= _dims[i];
		}
		// Flip to get mfactor
		for (unsigned int j= (unsigned int) mfactorflip.size()-1; j!=0; j--)
		{
			mfactor.push_back(mfactorflip[j]);
		}
	for ( it = _vals.begin(); it != _vals.end(); it++)
	{
		unsigned int pos = 0;
		for (unsigned int k=0; k<_dims.size(); k++)
		{
			pos += mfactor[k] * it->first[k];
		}
		// We now have a position vector!
		// TODO: check for accurate math
		
		// Finally, place the value in the array
		target[pos] = it->second;
	}

}

void matrixop::fromDoubleArray(const double *target)
{
	// Assume that the double array is of matching size as the matrixop
	// TODO: implement subset adding / extraction
	// TODO: implement subsetting matrixop
	// Calculate max size (mult. of all dim sizes)


	// TODO: add a modular math class (to do these conversions)!!!!!
	using namespace std;
	unsigned int maxsize = 1;
	for (unsigned int i=0;i<_dims.size();i++)
	{
		maxsize *= _dims[i];
	}
	// Take in each value over [0,maxsize) and place in the matrix
	clear();

	throw;
	double cval = 0.0;
	for (unsigned int j=0;j<maxsize;j++)
	{
		cval = target[j];
		// Convert j into pos coords
	}
}

}; // end rtmath

