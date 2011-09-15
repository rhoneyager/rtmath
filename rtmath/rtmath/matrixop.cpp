#include "Stdafx.h"
#include "matrixop.h"
#include "error.h"
#include <memory>
// Reduces tedium when finding minors
//#include <boost/foreach.hpp>
#include <cmath>

namespace rtmath {

	void matrixop::_init(const std::vector<unsigned int> &size)
	{
		_data = NULL; // it is safe to put this here
		_datasize = 0; // just to give it a value for now
		_dims = size;
		// Allocate necessary memory for data
		size_t sz = maxSize();
		_realloc(sz);
	}

	matrixop::matrixop(const std::vector<unsigned int> &size)
	{
		_init(size);
	}

	matrixop::~matrixop(void)
	{
		_free();
	}

	matrixop::matrixop(const matrixop & rhs)
	{
		// The matrixop copy constructor
		_init(rhs._dims);

		// _datasize now holds the number of elements to directly copy
		// Do a deep copy of the memory contents
		// Iterate and zero the data
		for (size_t i=0; i<_datasize; i++)
			_data[i] = rhs._data[i];
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

		_init(ptr);
	}

	void matrixop::_realloc(size_t numelems)
	{
		// Do not preserve existing data
		// A matrix resize makes it pointless to do so
		if (_data) _free();
		_data = new double[numelems];
		_datasize = numelems;

		double *ptr = &_data[0];
		// Iterate and zero the data
		for (size_t i=0; i<numelems; i++)
		{
			*ptr = 0.0;
			ptr++;
		}
	}

	void matrixop::_free()
	{
		if (_data)
		{
			delete[] _data;
			_datasize = 0;
		}
	}

	void matrixop::resize(const std::vector<unsigned int> &size)
	{
		if (_data) _free();
		_init(size);
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

		resize(ptr);
	}

	matrixop* matrixop::clone() const
	{
		return new matrixop(*this);
	}

	size_t matrixop::dimensionality() const
	{
		return _dims.size();
	}

	void matrixop::size(std::vector<unsigned int> &out) const
	{
		out = _dims;
	}

	const std::vector<unsigned int> matrixop::size() const
	{
		return _dims;
	}

	matrixop matrixop::operator+ (const matrixop& rhs) const
	{
		TASSERT(rhs.dimensionality() == this->dimensionality());
		TASSERT(rhs.size() == this->_dims);

		matrixop temp(_dims);
		temp = rhs;

		for (size_t i = 0; i < _datasize; i++)
		{
			temp._data[i] = this->_data[i] + rhs._data[i];
		}
		return temp;
	}

	matrixop matrixop::operator* (const matrixop& rhs) const
	{
		// Define standard matrix multiplication
		// That is: do the bilinear mapping between two rank 0,1 or 2 tensors
		// If greater dimensionality, throw an error since it's unsupported
		TASSERT(rhs.dimensionality() < 3);
		TASSERT(this->dimensionality() < 3);

		// size()[0] is number of rows, size()[1] is # of columns
		// If matrices cannot produce a square matrix, throw
		//std::vector<unsigned int> testA = this->size(), testB = rhs.size();
		TASSERT(this->size()[1] == rhs.size()[0]);

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
		TASSERT(rhs.dimensionality() == this->dimensionality());
				TASSERT(rhs.size() == this->_dims);

				matrixop temp(_dims);

				for (size_t i = 0; i < _datasize; i++)
				{
					temp._data[i] = this->_data[i] * rhs;
				}
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
		TASSERT(issquare());
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
		//throw;
	}

	matrixop matrixop::operator- (const matrixop& rhs) const
	{
		TASSERT(rhs.dimensionality() == this->dimensionality());
				TASSERT(rhs.size() == this->_dims);

				matrixop temp(_dims);
				temp = rhs;

				for (size_t i = 0; i < _datasize; i++)
				{
					temp._data[i] = this->_data[i] - rhs._data[i];
				}
				return temp;
	}

	void matrixop::clear()
	{
		//_vals.clear();
		for (size_t i=0; i<_datasize; i++)
			_data[i] = 0.0;
	}

	void matrixop::set(const std::vector<unsigned int> &pos, double val)
	{
		// Check for going beyond matrix bounds
		TASSERT(pos.size() == _dims.size());
		for (unsigned int i=0; i<pos.size(); i++)
		{
			if (pos[i] > _dims[i]) throw rtmath::debug::xBadInput();
		}

		size_t index;
		indexFromPos(index, pos);

		_data[index] = val;
	}

	void matrixop::set(double val, unsigned int rank, ...)
	{
		// Uses variadic array to get the index, as matrices can have multiple dimensions
		TASSERT(rank == this->dimensionality());
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
		TASSERT(rank == this->dimensionality());
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
		size_t index;
		indexFromPos(index, pos);

		return _data[index];
	}

	void matrixop::indexFromPos(size_t &index, std::vector<unsigned int> pos) const
	{
		// Check matrix bounds
		TASSERT(pos.size() == _dims.size());
		for (unsigned int i=0; i<pos.size(); i++)
		{
			if (pos[i] > _dims[i]) throw rtmath::debug::xBadInput();
		}

		using namespace std;
		unsigned int maxsize = _datasize;

		index = 0; // The index position
		size_t runningMult = 1; // A running multiplier to work with the vector index calculation
		// Working from the end of the position vector
		vector<unsigned int>::reverse_iterator rit;
		vector<unsigned int>::const_reverse_iterator sit;
		sit = this->_dims.rbegin();
		for (rit = pos.rbegin(); rit < pos.rend(); ++rit)
		{
			index += runningMult * *rit;
			runningMult *= *sit;
			++sit;
		}
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
		for (size_t i=0; i < _datasize; i++)
		{
			if (_data[i] != rhs._data[i]) return false;
		}

		return true;
	}

	matrixop matrixop::minors(unsigned int rank, ...) const
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
		return this->minors(ptr);
	}

	void matrixop::set(double val, unsigned int index)
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
		TASSERT(dimensionality());
		pos.clear();
		pos.resize(_dims.size());
		pos[_dims.size()-1] = index;

		// If > dimensionality, do modular addition
		// BOOST_REVERSE_FOREACH(unsigned int i,pos) // REVERSE_FOREACH seems unreliable
		for (int i=(int) _dims.size()-1;i>=0;i--)
		{
			if (pos[i] >= _dims[i] && i > 0)
			{
				pos[i-1] += pos[i] / _dims[i];
				pos[i] = pos[i] % _dims[i];
			}
		}
	}

	matrixop matrixop::minors(const std::vector<unsigned int> &rc) const
	{
		using namespace std;
		TASSERT(_dims.size() == 2); // Allow only 2d matrices
		vector<unsigned int> msiz = _dims;
		for (unsigned int i=0;i<msiz.size();i++)
		{
			msiz[i] = msiz[i] - 1;
			if (msiz[i] == 0) throw rtmath::debug::xBadInput();
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
		TASSERT(issquare());
		if (dimensionality() == 1) return get(1,0);
		// Check that matrix is 2d, for now
		// TODO: extend this to n dims, if possible
		TASSERT(_dims.size() == 2);

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
			matrixop min = this->minors(pos);
			// () alternates negatives, get(it) is the value at (0,i) and min.det is the minor's det
			res += std::pow(-1.0, (double) it[1]) * min.det() * get(it);
		}
		return res;
	}

	matrixop matrixop::diagonal(const std::vector<unsigned int> &size, double val)
	{
		matrixop res(size);
		TASSERT(size.size() == 2);
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
		TASSERT(mdet);
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
				matrixop min = this->minors(pos);
				double inv = std::pow(-1.0, (double) (i+j) ) * min.det() / mdet;
				res.set(inv,2,j,i);
			}
		}
		return res;
	}
/*
	matrixop matrixop::inverse() const
	{
		// First, calculate the determinant
		// It's going here because I'm using gaussian elimination to speed things up,
		// and I don't want to do the same operations twice.
		// Loosely inspired from Evans' code (same method, different language, different implementation)
		if (this->issquare() == false) throw rtmath::debug::xBadInput();

		double det = 0;
		matrixop res(*this);
		matrixop inv = matrixop::diagonal(_dims,1.0);
		// Repivot matrix su
		// Use gaussian elimination to first produce an upper triangular matrix
		for (size_t row=0; row<_dims[0]; row++)
		{
			// Iterate through each row, and subtract
			// If the desired row is missing the necessary pivot, search for a row to swap with
			// - if no swapping row is found, the matrix is singular and throw an error
			// Note: must also repivot the identity matrix to match
			if (res.get(2,row,row) == 0) // repivot if necessary
			{
				size_t rowb;
				rowb = _repivot(res,res,row);
				_swaprows(inv,row,rowb); // perform the swap on the inverse matrix as well
			}

			// Next, subtract a multiple of this row from the subsequent rows
			for (size_t rowb=row+1; rowb<_dims[0]; rowb++)
			{
				double multval = res.get(2,rowb,row) / res.get(2,row,row);

			}
		}
		// Then, go backwards to make the
	}
*/

	void matrixop::upperTriangular(matrixop &res) const
	{
		// Very useful for calculating determinants and inverses
		if (this->issquare() == false) throw rtmath::debug::xBadInput();
		res = *this;
		size_t numRows = _dims[0];
		size_t numCols = _dims[1];
		matrixop secondary = diagonal(_dims,1.0); // To keep track of inverse
		// Repivot matrix su
		// Use gaussian elimination
		for (size_t row=0; row<_dims[0]; row++)
		{
			// Iterate through each row, and subtract
			// If the desired row is missing the necessary pivot, search for a row to swap with
			// - if no swapping row is found, the matrix is singular and throw an error
			// Note: must also repivot the identity matrix to match
			if (res.get(2,row,row) == 0) // repivot if necessary
			{
				size_t rowb;
				rowb = _repivot(res,res,row);
				_swaprows(secondary,row,rowb); // perform the swap on the secondary matrix as well
			}

			// Next, subtract a multiple of this row from the subsequent rows
			for (size_t rowb=row+1; rowb<_dims[0]; rowb++)
			{
				double multval = res.get(2,rowb,row) / res.get(2,row,row);
				for (size_t col=0; col<numCols; col++)
				{
					res.set( res.get(2,rowb,col) - (multval * res.get(2,row,col)) ,2,rowb,col);
					secondary.set( secondary.get(2,rowb,col) - (multval * secondary.get(2,row,col)) ,2,rowb,col);
				}
			}
		}
	}

	void matrixop::_swaprows(matrixop &source, size_t rowa, size_t rowb)
	{
		// Fast relinking function that swaps two rows using internal functions
		// Check row bounds
		if (source._dims[0] < rowa || source._dims[0] < rowb) throw rtmath::debug::xBadInput();
		// The values are all actually in a map of position vectors.
		// This is unfortunate, as it may slow the code.
		// Create a map of all elements in rows a and b
		size_t rows = source._dims[0];
		size_t cols = source._dims[1];
		size_t size = rows * cols;
		double * elems = new double[size];
		source.toDoubleArray(elems);
		// Do the swap of the rows
		double *a = &elems[cols*rowa];
		double *b = &elems[cols*rowb];
		for (size_t i=0; i < cols; i++)
		{
			double holder = *b;
			*b = *a;
			*a = holder;
			a++;
			b++;
		}

		// Reread from elems to get swapped matrix
		source.fromDoubleArray(elems);
		delete[] elems;
	}

	size_t matrixop::_repivot(const matrixop &source, matrixop &res, size_t row)
	{
		// Search below for a row that can be swapped with the current row to ensure upper triangularity can be preserved
		std::vector<unsigned int> pos;
		pos.push_back(row+1);
		pos.push_back(row);
		res = source;
		for (size_t i = row++; i<res._dims[0]; i++, pos[0]++)
		{
			if (res.get(pos) != 0)
			{
				// We have a match
				_swaprows(res,row,pos[0]);
				return pos[0];
			}
		}
		throw; // TODO: Give a throw class. Occurs when matrix cannot be fully decomposed
	}


	matrixop matrixop::identity(const std::vector<unsigned int> &size)
	{
		return diagonal(size,1.0);
	}

	void matrixop::toDoubleArray(double *target) const
	{
		// Take the data and convert to an array of doubles
		// Use pointers to do this, and assume that target is pre-sized appropriately
		for (size_t i=0; i<_datasize; i++)
			target[i] = _data[i];
	}

	void matrixop::print() const
	{
		TASSERT(_dims.size() == 2);
		using namespace std;
		for (unsigned int i=0;i<_dims[0];i++) // row
		{
			for (unsigned int j=0;j<_dims[1];j++) // column
				cout << get(2,i,j) << "\t";
			cout << endl;
		}
		cout << endl;
	}
	/*
	void matrixop::fromCdf(NcVar *var, long n)
	{
	// Calculate max size
	unsigned int maxsize = maxSize();
	// Clear matrixop of any values
	clear();
	// Initialize an array of doubles
	double *dvals = new double[maxsize];

	NcValues *vals = var->get_rec(n);
	for (long i=0; i< (long) maxsize; i++)
	{
	dvals[i] = vals->as_double(i);
	}

	fromDoubleArray(dvals);

	// Clean up
	delete[] dvals;
	}

	void matrixop::toCDF(NcVar *var) const
	{
	unsigned int maxsize = maxSize();
	double *dvals = new double[maxsize];

	toDoubleArray(dvals);

	var->put_rec(dvals);

	delete[] dvals;
	}
	*/
	unsigned int matrixop::maxSize() const
	{
		using namespace std;
		unsigned int maxsize = 1;
		for (unsigned int i=0;i<_dims.size();i++)
		{
			maxsize *= _dims[i];
		}
		return maxsize;
	}

	void matrixop::posFromIndex(size_t index, std::vector<unsigned int> &pos) const
	{
		pos.clear();
		pos.resize(_dims.size(),0);
		//unsigned int maxsize = maxSize();

		for (size_t j=0;j<index;j++)
		{
			//cval = target[j];
			// Convert j into pos coords
			//set(pos,cval);
			// Find the next pos
			unsigned int last = (unsigned int) pos.size() - 1;
			pos[last]++;
			while (last > 0)
			{
				if (pos[last] >= _dims[last]) 
				{
					pos[last-1]++;
					pos[last] = 0;
				}
				last--;
			}
			if (pos[0] > _dims[0]) throw rtmath::debug::xArrayOutOfBounds();
		}
	}

	void matrixop::fromDoubleArray(const double *target)
	{
		// Assume that the double array is of matching size as the matrixop
		for (size_t i=0; i<_datasize; i++)
			_data[i] = target[i];
	}

	matrixop matrixop::fileRead(const char* filename, int lineStart, int lineEnd)
	{
		// Open file. Seek starting line. Read through and including ending line.
		// Determine matrix dimensions. Convert double array into a matrixop.
		double val = 0.0;
		bool readtoeof = false;
		if (lineEnd < 0) readtoeof = true;
		using namespace std;
		vector<double> vals;
		ifstream indata(filename);
		if (!indata) throw rtmath::debug::xEmptyInputFile(filename);
		if (indata.good() == false) throw rtmath::debug::xEmptyInputFile(filename);
		if (indata.eof()) throw rtmath::debug::xEmptyInputFile(filename);
		if (indata.bad()) throw rtmath::debug::xEmptyInputFile(filename);
		if (indata.fail()) throw rtmath::debug::xEmptyInputFile(filename);
		string lineIn;
		// Seek to start
		for (int i=0; i<lineStart; i++)
			std::getline(indata, lineIn);

		int linesRead = 0;
		int colsRead=0;
		for (int i=lineStart; 
			( (i<lineEnd) || readtoeof) && indata.eof() == false; 
			i++, linesRead++)
		{
			std::getline(indata,lineIn);
			istringstream proc(lineIn);
			while (proc.eof() == false)
			{
				proc >> val;
				vals.push_back(val);
				if (linesRead == 0) colsRead++;
			}
		}

		// Convert the vector of values into a double array
		double *dvals = new double[vals.size()];
		double *cval = &dvals[0];
		vector<double>::const_iterator it;
		for (it = vals.begin(); it != vals.end(); it++, cval++)
			*cval = *it;
		
		// Construct matrixop from double array
		matrixop res(2, linesRead, colsRead);
		res.fromDoubleArray(dvals);
		delete[] dvals;
		return res;
	}

}; // end rtmath

