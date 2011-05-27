#include "matrixop.h"

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

std::vector<unsigned int> matrixop::size() const
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

