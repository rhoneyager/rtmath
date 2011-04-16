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
}

matrixop* matrixop::clone()
{
	return new matrixop(*this);
}

/* An example from earlier code
polynomial polynomial::operator+ (polynomial param)
	{
		polynomial temp;
		// Compare degree of both polynomials and pick max
		unsigned int maxDeg = maxPow();
		if (param.maxPow() > maxDeg) maxDeg = maxPow();
		for (unsigned int i=0;i<=maxPow();i++)
			temp.coeff(i, coeff(i) + param.coeff(i));
		temp.truncate();
		return temp;
	}
	*/

unsigned int matrixop::dimensionality() const
{
	return _dims.size();
}

void matrixop::size(std::vector<unsigned int> &out) const
{
	out = _dims;
}

std::vector<unsigned int>& matrixop::size() const
{
	//TODO: check to see if this code works as intended
	static std::vector<unsigned int> temp = _dims;
	return temp;
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
	if (this->size()[1] != rhs.size()[0]) throw;

	// Create vector for size of result
	using namespace std;
	vector<unsigned int> ressize;
	ressize.push_back(this->size()[0]); // number of rows
	ressize.push_back(rhs.size()[0]); // number of columns
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
				pb[1] = i;
				val += this->get(pa) * rhs.get(pb);
			}
			res.set(pos,val);
		}
	}
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
	if (_vals.count(pos) > 0) return _vals.at(pos);
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

}; // end rtmath

