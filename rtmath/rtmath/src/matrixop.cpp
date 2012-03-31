#include "../rtmath/Stdafx.h"
#include <memory>
#include <cmath>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"


namespace rtmath {

#ifndef USE_IPP

	void matrixop::_init(const std::vector<size_t> &size)
	{
		_data = NULL; // it is safe to put this here
		_datasize = 0; // just to give it a value for now
		_dims = size;
		// Allocate necessary memory for data
		size_t sz = maxSize();
		_realloc(sz);
	}

	matrixop::matrixop(const std::vector<size_t> &size)
	{
		_init(size);
	}

	matrixop::matrixop()
	{
		std::vector<size_t> sz;
		sz.push_back(1);
		sz.push_back(1);
		_init(sz);
	}

	matrixop::~matrixop(void)
	{
		//std::cerr << "destructing" << std::endl;
		_free();
	}

	matrixop::matrixop(const matrixop & rhs)
	{
		// The matrixop copy constructor
		_init(rhs._dims);

		// _datasize now holds the number of elements to directly copy
		// Do a deep copy of the memory contents
		for (size_t i=0; i<_datasize; i++)
			_data[i] = rhs._data[i];
	}

	matrixop & matrixop::operator=(const matrixop & rhs)
	{
		if (this == &rhs) return *this; // self-assignment check
		resize(rhs._dims);
		// _datasize now holds the number of elements to directly copy
		// Do a deep copy of the memory contents
		for (size_t i=0; i<_datasize; i++)
			_data[i] = rhs._data[i];
		return *this;
	}

	matrixop & matrixop::operator=(const double * rhs)
	{
		// Assign values from an array of doubles.
		// Essentially an alias to fromDoubleArray.
		fromDoubleArray(rhs);
		return *this;
	}

	matrixop::matrixop(size_t ndims, ...)
	{
		va_list indices;
		va_start(indices, ndims);
		std::vector<size_t> ptr;
		size_t ival;
		for (size_t i=0; i<ndims; i++)
		{
			ival = va_arg(indices,size_t);
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

		// Iterate and zero the data
		for (size_t i=0; i<numelems; i++)
		{
			_data[i] = 0.0;
		}
	}

	void matrixop::_free()
	{
		if (_data)
		{
			//for (size_t i=0; i<_datasize; i++) // debugging to find memory overwrite
			//_data[i] = -99;
			delete[] _data;
			_datasize = 0;
		}
	}

	void matrixop::resize(const std::vector<size_t> &size)
	{
		if (_data) _free();
		_init(size);
	}

	void matrixop::resize(size_t ndims, ...)
	{
		va_list indices;
		va_start(indices, ndims);
		std::vector<size_t> ptr;
		size_t ival;
		for (size_t i=0; i<ndims; i++)
		{
			ival = va_arg(indices,size_t);
			ptr.push_back(ival);
		}
		va_end(indices);

		resize(ptr);
	}

	/*matrixop* matrixop::clone() const
	{
		return new matrixop(*this);
	}*/

	size_t matrixop::dimensionality() const
	{
		return _dims.size();
	}

	void matrixop::size(std::vector<size_t> &out) const
	{
		out = _dims;
	}

	const std::vector<size_t> matrixop::size() const
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
		//std::vector<size_t> testA = this->size(), testB = rhs.size();
		TASSERT(this->size()[1] == rhs.size()[0]);

		// Create vector for size of result
		using namespace std;
		vector<size_t> ressize;
		ressize.push_back(this->size()[0]); // number of rows
		ressize.push_back(rhs.size()[1]); // number of columns
		matrixop res(ressize);

		// Actually calculate the resulting values
		// TODO: optimize this
		for (size_t i=0; i< res.size()[0]; i++)
		{
			for (size_t j=0; j< res.size()[1]; j++)
			{
				vector<size_t> pos(2,0), pa(2,0), pb(2,0);
				pos[0] = i;
				pos[1] = j;
				// Calculate res(i,j)
				double val = 0;
				for (size_t k=0; k<this->size()[1]; k++)
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
		return res;
	}

	matrixop matrixop::operator* (double rhs) const
	{
		matrixop temp(_dims);

		for (size_t i = 0; i < _datasize; i++)
		{
			temp._data[i] = this->_data[i] * rhs;
		}
		return temp;
	}

	matrixop matrixop::operator%(const matrixop& rhs) const
	{
		// Define standard elementwise matrix multiplication
		TASSERT(this->dimensionality() == rhs.dimensionality());
		for (size_t i=0;i<_dims.size();i++)
		{
			// Make sure all dimensions are the same
			TASSERT(_dims[i] == rhs._dims[i]);
		}

		using namespace std;
		matrixop res(_dims);
		for (size_t i=0;i<maxSize();i++)
		{
			res._data[i] = _data[i] * rhs._data[i];
		}

		return res;
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
			for (size_t i=0; i< _dims[0]; i++)
			{
				std::vector<size_t> pos(_dims[0],i);
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

	void matrixop::set(const std::vector<size_t> &pos, double val)
	{
		// Check for going beyond matrix bounds
		TASSERT(pos.size() == _dims.size());
		for (size_t i=0; i<pos.size(); i++)
		{
			if (pos[i] > _dims[i]) throw rtmath::debug::xBadInput("Beyond matrix bounds");
		}

		size_t index;
		indexFromPos(index, pos);

		_data[index] = val;
	}

	void matrixop::set(double val, size_t rank, ...)
	{
		// Uses variadic array to get the index, as matrices can have multiple dimensions
		TASSERT(rank == this->dimensionality());
		va_list indices;
		va_start(indices, rank);
		std::vector<size_t> ptr;
		size_t ival;
		for (size_t i=0; i<rank; i++)
		{
			ival = va_arg(indices,size_t);
			ptr.push_back(ival);
		}
		va_end(indices);
		set(ptr,val);
	}

	double matrixop::get(size_t rank, ...) const
	{
		// Uses variadic array to get the index, as matrices can have multiple dimensions
		TASSERT(rank == this->dimensionality());
		va_list indices;
		va_start(indices, rank);
		std::vector<size_t> ptr;
		size_t ival;
		for (size_t i=0; i<rank; i++)
		{
			ival = va_arg(indices,size_t);
			ptr.push_back(ival);
		}
		va_end(indices);
		return get(ptr);
	}

	double matrixop::get(const std::vector<size_t> &pos) const
	{
		size_t index;
		indexFromPos(index, pos);

		return _data[index];
	}

	void matrixop::indexFromPos(size_t &index, std::vector<size_t> pos) const
	{
		// Check matrix bounds
		TASSERT(pos.size() == _dims.size());
		for (size_t i=0; i<pos.size(); i++)
		{
			if (pos[i] > _dims[i]) throw rtmath::debug::xBadInput("Beyond matrix bounds");
		}

		using namespace std;
		//size_t maxsize = _datasize;

		index = 0; // The index position
		size_t runningMult = 1; // A running multiplier to work with the vector index calculation
		// Working from the end of the position vector
		vector<size_t>::reverse_iterator rit;
		vector<size_t>::const_reverse_iterator sit;
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

	void matrixop::minors(matrixop &res, size_t rank, ...) const
	{
		// Retrieve the variable parameters
		va_list indices;
		va_start(indices, rank);
		std::vector<size_t> ptr;
		size_t ival;
		for (size_t i=0; i<rank; i++)
		{
			ival = va_arg(indices,size_t);
			ptr.push_back(ival);
		}
		va_end(indices);
		this->minors(ptr,res);
	}

	void matrixop::set(double val, size_t index)
	{
		// Used by minor to place a value in the appropriate location
		using namespace std;
		vector<size_t> pos;
		_getpos(index,pos);
		// Set the value
		set(pos,val);
	}

	void matrixop::_getpos(size_t index, std::vector<size_t> &pos) const
	{
		using namespace std;
		TASSERT(dimensionality());
		pos.clear();
		pos.resize(_dims.size());
		pos[_dims.size()-1] = index;

		// If > dimensionality, do modular addition
		// BOOST_REVERSE_FOREACH(size_t i,pos) // REVERSE_FOREACH seems unreliable
		for (int i=(int) _dims.size()-1;i>=0;i--)
		{
			if (pos[i] >= _dims[i] && i > 0)
			{
				pos[i-1] += pos[i] / _dims[i];
				pos[i] = pos[i] % _dims[i];
			}
		}
	}

	void matrixop::minors(const std::vector<size_t> &rc, matrixop &res) const
	{
		using namespace std;
		TASSERT(_dims.size() == 2); // Allow only 2d matrices
		vector<size_t> msiz = _dims;
		for (size_t i=0;i<msiz.size();i++)
		{
			msiz[i] = msiz[i] - 1;
			if (msiz[i] == 0) throw rtmath::debug::xBadInput("Minor would be a null matrix");
		}
		res.resize(msiz);
		//matrixop res(msiz);
		// I have the empty resultant matrix
		// Now, to fill in the minors
		size_t rowX=rc[0],colX=rc[1];
		int rS = 0, cS = 0; // Row / column shift
		// Since this is used in more than just determinants, I cannot assume that
		// the matrix is square. Too bad.
		// Iterate over the rows and columns of the initial matrix
		for (size_t srow=0;srow<_dims[0];srow++)
		{
			if (srow == rowX) rS = 1; // Add the skip factor for future rows
			if (srow == rowX) continue; // Don't even bother here
			cS = 0;
			for (size_t scol=0;scol < _dims[1];scol++)
			{
				if (scol == colX) cS = 1;
				if (scol == colX) continue;
				// Do the actual value copying here
				double val = get(2,srow,scol);
				res.set(val,2,srow-rS,scol-cS); // Row and column skip factors shift the result!
			}
		}
		//return res;
	}

	/* // superseded by a method that computes an upper triangular matrix instead of finding minors
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


	vector<size_t> it;
	it.push_back(0); // 0th row
	it.push_back(0); // 0th column
	// Retrieve the first row of the matrix, get minor, and calculate determinant of minor
	double res = 0.0;
	// Wow, this next line is obtuse
	for (it[1] = 0; it[1] < _dims[1]; it[1]++)
	{
	std::vector<size_t> pos;
	pos.push_back(it[0]); pos.push_back(it[1]);
	//throw;
	matrixop min = this->minors(pos);
	// () alternates negatives, get(it) is the value at (0,i) and min.det is the minor's det
	res += std::pow(-1.0, (double) it[1]) * min.det() * get(it);
	}
	return res;
	}
	*/

	double matrixop::det() const
	{
		// Next line assumes square matrix. If not, it will throw an error.
		try {
			matrixop b(_dims);
			matrixop sec(_dims);
			upperTriangular(b, sec);
			// Iterate through the diagonal elements and calculate the determinant
			double res = 1.0;
			for (size_t i = 0; i < _dims[0]; i++)
				res *= b.get(_dims.size(), i, i);
			return res;
		}
		catch (rtmath::debug::xSingular)
		{
			return 0.0;
		}
	}

	matrixop matrixop::diagonal(const std::vector<size_t> &size, double val)
	{
		matrixop res(size);
		TASSERT(size.size() == 2);
		// Find min dimension
		size_t min = (size[0] < size[1])? size[0] : size[1];
		for (size_t i=0;i<min;i++)
		{
			res.set(val,2,i,i);
		}
		return res;
	}

	matrixop matrixop::diagonal(double val, size_t rank, ...)
	{
		// Retrieve the variable parameters
		va_list indices;
		va_start(indices, rank);
		std::vector<size_t> ptr;
		size_t ival;
		for (size_t i=0; i<rank; i++)
		{
			ival = va_arg(indices,size_t);
			ptr.push_back(ival);
		}
		va_end(indices);
		return diagonal(ptr,val);
	}

	/*
	matrixop matrixop::inverse() const
	{
		// This uses minors and a determinant to calculate the inverse
		double mdet = det();
		TASSERT(mdet);
		// Construct the matrix minors
		// Assumes 2d array from det. TODO: extend if possible
		matrixop res(this->_dims);
		for (size_t i=0;i<_dims[0];i++) // columns
		{
			for (size_t j=0;j<_dims[1];j++) // rows
			{
				std::vector<size_t> pos; pos.push_back(j); pos.push_back(i);
				// Calculate the appropriate minor
				//throw;
				matrixop min = this->minors(pos);
				double inv = std::pow(-1.0, (double) (i+j) ) * min.det() / mdet;
				res.set(inv,2,j,i);
			}
		}
		return res;
	}
	*/

	void matrixop::inverse(matrixop &inv) const
	{
		// Use gaussian elimination
		TASSERT(_dims.size() == 2);
		matrixop ut(_dims), invcheck(_dims);
		inv = diagonal(_dims,1.0);
		upperTriangular(ut, inv);
		ut.lowerTriangular(invcheck,inv);
		size_t numCols = _dims[1];
		// Go through rows, and perform row-based multiplication
		for (size_t i=0; i<_dims[0];i++)
		{
			double factor = invcheck._data[i*numCols+i];
			if (factor != 1.0) inv._rowmult(i, 1.0/factor);
		}
		//return inv;
	}
	/*
	void matrixop::expand(matrixop &res) const
	{
		// Expand current matrix to match the dimensions of res
		// This preserves dimensionality.

		std::vector<size_t> mysize, ressize;
		mysize = size();
		ressize = res.size();
		// Check that dimensions are compatible
		for (auto it = mysize.begin(), ot = ressize.begin(); it != mysize.end() && ot != ressize.end();
			++it, ++ot)
		{
			if (*ot < *it) throw debug::xArrayOutOfBounds();
		}

		res.clear();


		std::vector<size_t> ptr(ressize.size(), 1);
	}

	void matrixop::squeeze(matrixop &res) const
	{
	}

	void matrixop::subset(const std::vector<size_t> &start, const std::vector<size_t> &span, matrixop &res) const
	{
		res.resize(span);
		// Do modular addition
		std::vector<size_t> offset(span.size());

	}

	void matrixop::superset(const std::vector<size_t> start, matrixop &res) const
	{
	}
	*/
	void matrixop::_rowmult(size_t row, double factor)
	{
		// Multiply a given row by a given factor
		TASSERT(_dims.size() == 2);
		std::vector<size_t> pos;
		pos.push_back(row);
		pos.push_back(0);
		size_t indexStart;
		indexFromPos(indexStart, pos);
		for (size_t i=0; i<_dims[1];i++) // iterate over all columns in this row
			_data[indexStart+i] *= factor;
	}

	void matrixop::upperTriangular(matrixop &res, matrixop &secondary) const
	{
		// Very useful for calculating determinants and inverses
		if (this->issquare() == false) 
			throw rtmath::debug::xBadInput("upperTriangular code written for square matrices only.");
		//res = *this; // wrong clone constructor - produces pointer errors
		try {
			res.resize(_dims);
			this->toDoubleArray(res._data);

			//size_t numRows = _dims[0];
			size_t numCols = _dims[1];
			//matrixop secondary = diagonal(_dims,1.0); // To keep track of inverse
			// Repivot matrix su
			// Use gaussian elimination
			for (size_t row=0; row<_dims[0]; row++)
			{
				// Iterate through each row, and subtract
				// If the desired row is missing the necessary pivot, search for a row to swap with
				// - if no swapping row is found, the matrix is singular and throw an error
				// Note: must also repivot the identity matrix to match
				double val = res.get(2,row,row);
				if (val == 0) // repivot if necessary
				{
					size_t rowb;
					rowb = _repivot(res,row, false);
					_swaprows(secondary,row,rowb); // perform the swap on the secondary matrix as well
				}

				// Next, subtract a multiple of this row from the subsequent rows
				for (size_t rowb=row+1; rowb<_dims[0]; rowb++)
				{
					double multval = res.get(2,rowb,row) / res.get(2,row,row);
					for (size_t col=0; col<numCols; col++)
					{
						std::vector<size_t> p; // why not? it's an artifact from debugging that is still quite valid
						p.push_back(rowb); p.push_back(col);
						res.set( p, res.get(2,rowb,col) - (multval * res.get(2,row,col)));
						secondary.set( secondary.get(2,rowb,col) - (multval * secondary.get(2,row,col)) ,2,rowb,col);
					}
				}
			}
		}
		catch (rtmath::debug::xSingular)
		{
			throw rtmath::debug::xSingular();
		}
	}

	void matrixop::lowerTriangular(matrixop &res, matrixop &secondary) const
	{
		// Very useful for calculating determinants and inverses
		if (this->issquare() == false) 
			throw rtmath::debug::xBadInput("lowerTriangular code written for square matrices only");

		try {
			//res = *this; // wrong clone constructor - produces pointer errors
			res.resize(_dims);
			this->toDoubleArray(res._data);

			//size_t numRows = _dims[0];
			size_t numCols = _dims[1];
			//matrixop secondary = diagonal(_dims,1.0); // To keep track of inverse
			// Repivot matrix su
			// Use gaussian elimination
			for (size_t row=_dims[0]-1; row<_dims[0]; row--) // overflow will end the loop!
			{
				// Iterate through each row, and subtract
				// If the desired row is missing the necessary pivot, search for a row to swap with
				// - if no swapping row is found, the matrix is singular and throw an error
				// Note: must also repivot the identity matrix to match
				double val = res.get(2,row,row);
				if (val == 0) // repivot if necessary
				{
					size_t rowb;
					rowb = _repivot(res,row, true);
					_swaprows(secondary,row,rowb); // perform the swap on the secondary matrix as well
				}

				// Next, subtract a multiple of this row from the subsequent rows
				for (size_t rowb=row-1; rowb<_dims[0]; rowb--) // again, counting on overflow
				{
					double multval = res.get(2,rowb,row) / res.get(2,row,row);
					for (size_t col=0; col<numCols; col++)
					{
						std::vector<size_t> p; // why not? it's an artifact from debugging that is still quite valid
						p.push_back(rowb); p.push_back(col);
						res.set( p, res.get(2,rowb,col) - (multval * res.get(2,row,col)));
						secondary.set( secondary.get(2,rowb,col) - (multval * secondary.get(2,row,col)) ,2,rowb,col);
					}
				}
			}
		}
		catch (rtmath::debug::xSingular)
		{
			throw rtmath::debug::xSingular();
		}
	}

	void matrixop::transpose(matrixop &res) const
	{
		// Calculates the transpose of the matrix, and places it in res
		// TODO: speed / figure out pattern to avoid 

		std::vector<size_t> dt;
		dt.push_back(_dims[1]);
		dt.push_back(_dims[0]);
		res.resize(dt);

		// Iterate through the index, and convert this into a vector
		// Then, set res with the reversed vector elements
		for (size_t i = 0; i < _datasize; i++)
		{
			std::vector<size_t> pos;
			posFromIndex(i, pos);
			res.set( _data[i], 2, pos[1], pos[0]);
		}
	}

	void matrixop::HouseholderUT(matrixop &target) const
	{
		// If matrix is symmetric, this tridiagonalizes it
		// Constructing the upper hessenberg matrix is useful for solving
		// for eigenvalues, which is used extensively in zero-finding code

		// Uses Householder reductions
		// H = I - 2 u uT


		// Can be done on a non-square matrix!
		std::vector<matrixop> results;

		matrixop wrk(_dims);
		wrk = *this;

		// Loop through for number or rows times
		for (size_t i=0; i<_dims[0]-1; i++)
		{
			std::vector<matrixop> as, vs, Has;
			std::vector<double> fs;
			matrixop wrknext(wrk._dims);
			//std::cout << "wrk\n"; wrk.print();

			double a11 = wrk.get(2,0,0);
			matrixop a1(2,wrk._dims[0],1);

			for (size_t j=0; j<wrk._dims[1];j++)
			{
				// Read down the column and define a
				matrixop a(2,wrk._dims[0],1), v(2,wrk._dims[0],1), Ha(2,wrk._dims[0],1);
				for (size_t k=0; k<wrk._dims[0];k++)
					a.set( wrk.get(2,k,j) , 2, k, 0);
				if (j==0) a1 = a;
				double d;
				double a1m = sqrt((a1.transpose() * a1).get(2,0,0));
				//double a11 = a.get(2,0,0);
				(a11>0) ? d = -std::abs(a1m) : d = std::abs(a1m);
				double w11 = a11 - d;
				double f1 = sqrt(-2.0*w11*d);
				v.set(w11 / f1, 2, 0,0);
				for (size_t k=1;k<wrk._dims[0];k++)
					v.set(wrk.get(2,k,0) / f1, 2, k, 0);
				double fi;
				(j==0) ? fi = f1 : fi = 2.0 * (v.transpose()*a).get(2,0,0);
				if (j == 0)
				{
					Ha.set(d,2,0,0);
				} else {
					Ha = a - (v * fi);
				}
				//std::cout << "Ha\n"; Ha.print();
				Has.push_back(Ha);

			}

			// Create result matrix
			for (size_t j=0; j< Has.size(); j++)
			{
				wrknext.setCol(j,Has[j]._data);
			}
			results.push_back(wrknext);
			// Set work equal to its minor
			matrixop newwrk(2,wrk._dims[0] - 1, wrk._dims[1]);
			wrknext.minors(newwrk, 2, 0, 0);
			wrk = newwrk; // and do the assignment
			//wrknext.print();
		}

		//for (size_t l=0;l<results.size();l++)
			//results[l].print();
		target.resize(_dims);
		// Now go through all results and place the elements in the appropriate places
		for (size_t i=0; i<_dims[0]; i++)
			for (size_t j=0; j<_dims[1]; j++)
			{
				// Get val from correct matrix and place in target
				double val = 0;
				size_t index = 0;
				(i>j) ? index = j : index = i; // take the minimum
				//std::cout << index << " " << i << " " << j << std::endl;
				//if (index >=results.size()) continue;
				if (index >= results.size()) index = results.size() - 1; // last box is really previous
				val = results[index].get(2, i-index, j-index);
				target.set(val,2,i,j);
			}
		//std::cout << std::endl << std::endl;
		//target.print();
	}

	void matrixop::upperHessenberg(matrixop &target) const
	{
		// If matrix is symmetric, this tridiagonalizes it
		// Constructing the upper hessenberg matrix is useful for solving
		// for eigenvalues, which is used extensively in zero-finding code

		// Uses Householder reductions
		// H = I - 2 u uT


		// Can be done on a non-square matrix!
		std::vector<matrixop> results;

		matrixop wrk(_dims);
		wrk = *this;

		// Loop through for number or rows times
		for (size_t i=0; i<_dims[0]-1; i++)
		{
			std::vector<matrixop> as, vs, Has;
			std::vector<double> fs;
			matrixop wrknext(wrk._dims);
			//std::cout << "wrk\n"; wrk.print();

			double a11 = wrk.get(2,0,0);
			matrixop a1(2,wrk._dims[0],1);

			for (size_t j=0; j<wrk._dims[1];j++)
			{
				// Read down the column and define a
				matrixop a(2,wrk._dims[0],1), v(2,wrk._dims[0],1), Ha(2,wrk._dims[0],1);
				for (size_t k=0; k<wrk._dims[0];k++)
					a.set( wrk.get(2,k,j) , 2, k, 0);
				std::cout << "j=" << j << ": a\n"; a.print();
				if (j==0) a1 = a;
				double d11;
				double a1ms = (a1.transpose() * a1).get(2,0,0) - (a11*a11);
				double a1m = sqrt(a1ms);
				//double a11 = a.get(2,0,0);
				(a11>0) ? d11 = -std::abs(a1m) : d11 = std::abs(a1m);
				std::cout << "d11 " << d11 << std::endl;
				double w11 = a11 - d11;
				double f1 = sqrt(-2.0*w11*d11);
				v.set(w11 / f1, 2, 1,0);
				for (size_t k=2;k<wrk._dims[0];k++)
					v.set(wrk.get(2,k,0) / f1, 2, k, 0);
				std::cout << "v\n"; v.print();
				double fi;
				(j==0) ? fi = f1 : fi = 2.0 * (v.transpose()*a).get(2,0,0);
				if (j == 0)
				{
					Ha.set(a11,2,0,0);
					Ha.set(d11,2,1,0);
					Ha.print();
				} else {
					Ha = a - (v * fi);
				}
				std::cout << "Ha\n"; Ha.print();
				Has.push_back(Ha);

			}

			// Create result matrix
			for (size_t j=0; j< Has.size(); j++)
			{
				wrknext.setCol(j,Has[j]._data);
			}
			results.push_back(wrknext);
			// Set work equal to its minor
			matrixop newwrk(2,wrk._dims[0] - 1, wrk._dims[1]);
			wrknext.minors(newwrk, 2, 0, 0);
			wrk = newwrk; // and do the assignment
			//wrknext.print();
		}

		//for (size_t l=0;l<results.size();l++)
			//results[l].print();
		target.resize(_dims);
		// Now go through all results and place the elements in the appropriate places
		for (size_t i=0; i<_dims[0]; i++)
			for (size_t j=0; j<_dims[1]; j++)
			{
				// Get val from correct matrix and place in target
				double val = 0;
				size_t index = 0;
				(i>j) ? index = j : index = i; // take the minimum
				//std::cout << index << " " << i << " " << j << std::endl;
				//if (index >=results.size()) continue;
				if (index >= results.size()) index = results.size() - 1; // last box is really previous
				val = results[index].get(2, i-index, j-index);
				target.set(val,2,i,j);
			}
		//std::cout << std::endl << std::endl;
		//target.print();
	}


	void matrixop::decompositionQR(matrixop &Q, matrixop &R) const
	{
		// Perform a QR decomposition of the matrix (hoping that it is invertible)
		// Matrix R is an upper right triangular matrix. 
		// Matrix Q is an orthogonal matrix.
		// This method uses Graham-Schmidt orthogonalization.
		// Inner products assume transposition, and since matrixop takes only real values, 
		// for now, does not compute complex conjugates.

		// First, extract the columns of the matrix, and put these into matrices of their own
		// I'm not using matrixops here, but instead stl vectors to speed things up
		// (only a subset of functionality is needed).

		std::vector< std::vector<double> > columns, u, e;
		size_t numCols = this->_dims[1];
		size_t numRows = this->_dims[0];
		columns.resize(numCols);
		u.resize(numCols);
		e.resize(numCols);
		for (size_t index = 0; index < _datasize; index++)
		{
			size_t column = index % numCols;
			columns[column].push_back(_data[index]);
		}

		// Now, to perform orthogonalization
		// projection is defined as proj_e(a) = (<e,a>/<e,e>) e
		// <a,b> = a^T b
		// e_i = u_i / ||u_i||
		// A is the full matrix, so a_i is the ith column in columns
		//std::vector<double>::const_iterator it;
		for (size_t i=0; i < numCols; i++) // i is the column number
		{
			// Calculate relevant projections
			// Projections fit into an (i-1)x(numRows) matrix
			double *proj;
			if (i > 0)
			{
				proj = new double[(i)*numRows];
			} else {
				proj = 0; // I won't use it anyways
			}
			// The calculation step for projections
			for (size_t j=0;j<i;j++)
			{
				// I'm arranging this into groups of proj_e_i (so the row elements are contiguous)
				//long double ee = 0; // since ||e|| = 1 by def, ee = 1, so discard.
				long double ea = 0;
				for (size_t k=0; k< numRows; k++)
				{
					double ejk = e[j][k];
					//ee += ejk * ejk; // should exist from previous iterations of the outermost loop
					ea += ejk * columns[i][k]; // careful about indices, here. col. i, not j!!!
				}
				for (size_t k=0; k<numRows;k++)
				{
					long double projval = (ea) * e[j][k];
					proj[(numRows*j)+k] = (double) projval;
				}
			}

			// Calculate u_i
			long double usq = 0;
			for (size_t k=0; k<numRows ;k++) // k is the element number in the column
			{
				double uk = columns[i][k]; // u_i = a_i - ... (first term)
				// Now, loop through and subtract the projections
				for(size_t j=0; j<i; j++)
				{
					// Projection was calculated above
					uk -= proj[(numRows*j)+k];
				}
				usq += uk*uk;
				u[i].push_back(uk);
			}
			// Calculate e_i (waited for ui to be complete)
			for (size_t k=0; k<numRows; k++)
			{
				// usq gives the norm
				e[i].push_back((double) u[i][k] / (double) sqrt(usq));
			}

			// Dump projection table (no longer needed)
			if (proj) delete[] proj;
		}


		// The orthogonalization step is done. Now, set Q to be the set of e_i columns.
		Q.resize(_dims);
		for (size_t col=0; col<numCols; col++)
			Q.setCol(col, e[col]);

		// Then, R=Q^T * A, which is easy to calculate
		matrixop Qt = Q.transpose();
		matrixop A = *this; // Otherwise, compile complains, even though matrixop * is defined as const... why?
		R = Qt * A;
	}
	/*
	void matrixop::QRalgorithm(matrixop &res, std::vector<double> &evals) const
	{
		// Performs the implicit QR algorithm to extract eigenvalues from a system
		// Uses shifting to speed up convergence
		// Assume A^(k) is symmetric throughout the process
		// As k->infty, the last col and row, except for A_nn^(k) approach zero
		// A_nn then is the lowest eigenvalue, and the matrix minor can be taken to
		// iterate for the next eigenvalue
		// Convergence is cubic
		throw rtmath::debug::xUnimplementedFunction();
	}
	*/
	void matrixop::setCol(size_t col, const std::vector<double> &data)
	{
		for (size_t i = col, j=0; i<_datasize; i+= _dims[1], j++)
		{
			_data[i] = data[j];
		}
	}

	void matrixop::setCol(size_t col, const double *data)
	{
		for (size_t i = col, j=0; i<_datasize; i+= _dims[1], j++)
		{
			_data[i] = data[j];
		}
	}

	void matrixop::setRow(size_t row, const std::vector<double> &data)
	{
		size_t start = _dims[1] * row;
		double *pt = &_data[start];
		std::vector<double>::const_iterator it;
		for (it = data.begin(); it != data.end(); it++)
		{
			*pt = *it;
			pt++;
		}
	}

	void matrixop::setRow(size_t row, const double *data)
	{
		size_t start = _dims[1] * row;
		double *pt = &_data[start];
		for (size_t i=0; i<_dims[1];i++)
		{
			*pt = data[i];
			pt++;
		}
	}

	void matrixop::_swaprows(matrixop &source, size_t rowa, size_t rowb)
	{
		// Fast relinking function that swaps two rows using internal functions
		// Check row bounds
		if (source._dims[0] < rowa || source._dims[0] < rowb) 
			throw rtmath::debug::xBadInput("Row swaps out of bounds");
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

	size_t matrixop::_repivot(matrixop &res, size_t row, bool reverse)
	{
		// Search below for a row that can be swapped with the current row to ensure upper triangularity can be preserved
		if (!reverse)
		{
			std::vector<size_t> pos;
			pos.push_back(row+1);
			pos.push_back(row);
			//res = source; // BAD
			for (size_t i = row+1; i<res._dims[0]; i++, pos[0]++)
			{
				double val = res.get(pos);
				if (val != 0.0)
				{
					// We have a match
					_swaprows(res,row,pos[0]);
					return pos[0];
				}
			}
		} else {
			std::vector<size_t> pos;
			pos.push_back(row-1);
			pos.push_back(row);
			//res = source; // BAD
			for (size_t i = row-1; i<res._dims[0]; i--, pos[0]--)
			{
				double val = res.get(pos);
				if (val != 0.0)
				{
					// We have a match
					_swaprows(res,row,pos[0]);
					return pos[0];
				}
			}
		}
		// TODO: instead, put all zero rows at the bottom (or top, if reversed) of the matrix
		throw rtmath::debug::xSingular();
		// Throw class needed for determinant and inverses to catch the singularity!!!!!
		return 0; // to force control path to return a value
	}


	matrixop matrixop::identity(const std::vector<size_t> &size)
	{
		return diagonal(size,1.0);
	}

	matrixop matrixop::identity(size_t rank, ...)
	{
		// Retrieve the variable parameters
		va_list indices;
		va_start(indices, rank);
		std::vector<size_t> ptr;
		size_t ival;
		for (size_t i=0; i<rank; i++)
		{
			ival = va_arg(indices,size_t);
			ptr.push_back(ival);
		}
		va_end(indices);
		return diagonal(ptr,1.0);
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
		for (size_t i=0;i<_dims[0];i++) // row
		{
			for (size_t j=0;j<_dims[1];j++) // column
				cout << get(2,i,j) << "\t";
			cout << endl;
		}
		cout << endl;
	}
	/*
	void matrixop::fromCdf(NcVar *var, long n)
	{
	// Calculate max size
	size_t maxsize = maxSize();
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
	size_t maxsize = maxSize();
	double *dvals = new double[maxsize];

	toDoubleArray(dvals);

	var->put_rec(dvals);

	delete[] dvals;
	}
	*/
	size_t matrixop::maxSize() const
	{
		using namespace std;
		size_t maxsize = 1;
		for (size_t i=0;i<_dims.size();i++)
		{
			maxsize *= _dims[i];
		}
		return maxsize;
	}

	void matrixop::posFromIndex(size_t index, std::vector<size_t> &pos) const
	{
		pos.clear();
		pos.resize(_dims.size(),0);
		//size_t maxsize = maxSize();

		for (size_t j=0;j<index;j++)
		{
			//cval = target[j];
			// Convert j into pos coords
			//set(pos,cval);
			// Find the next pos
			size_t last = (size_t) pos.size() - 1;
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

#endif // ifndef USE_IPP

}; // end rtmath
//
// istream / ostream overrides, used for putting / setting matrices
std::ostream & operator<<(std::ostream &stream, const rtmath::matrixop &ob)
{
	// Quick and easy output for now
	size_t n = ob.maxSize();
	double *data = new double[n];
	ob.toDoubleArray(data);
	stream << "{ ";
	for (size_t i=0;i<n-1;i++)
		stream << data[i] << ", ";
	stream << data[n-1] << " }";
	return stream;
}

// TODO: make sure that all istreams advance the stream!!!!

std::istream & operator>>(std::istream &stream, rtmath::matrixop &ob)
{
	// Input can be given either as just a stream of doubles or as 
	// a formatted set of blocks for parsing.
	// Examples: [[2,4,5][1,4,3][9,3,2]], {1,2,3,4,5,6,7,9,8}.

	// Do parsing of a set of values, removing any prefixes, suffixes
	// Split commas
	// Make sure that the number of values read matches the matrix
	// Perform readDoubleArray(...) to insert the data.
	throw rtmath::debug::xUnimplementedFunction();
	return stream;
}


