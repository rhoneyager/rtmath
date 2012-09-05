#pragma once
#include <algorithm>
#include <vector>
#include <cstdarg>
#include <iostream>

// Forward declaration for boost::serialization below
namespace rtmath {
	class matrixop;
}

// Need these so the template friends can work
namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive &, rtmath::matrixop &, const unsigned int);
		template <class Archive>
		void load(Archive &, rtmath::matrixop &, const unsigned int);
		template <class Archive>
		void save(Archive &, const rtmath::matrixop &, const unsigned int);
	}
}

#define inlinerval(x,y) inline x y(size_t rank, ...) const \
	{ x res(_dims); va_list indices; va_start(indices, rank); std::vector<size_t> ptr; \
	size_t ival; for (size_t i=0; i<rank; i++) { ival = va_arg(indices,size_t); \
	ptr.push_back(ival); } va_end(indices); y(ptr,res); return res; }

namespace rtmath {

	class matrixop
		// Defines a generalized set of tensor operations
		// For now, just use it as a 2d matrix
		//  More dimensions becomes more complex and should
		//  be saved for a more general base class
		// Useful because I can overload this and redefine matrix multiplication
		// This enables arbitrary multiplication / integration
		// It's a base class so I can reuse the code in many different ways
	{
		template<class Archive> 
		friend void ::boost::serialization::serialize(
			Archive &, matrixop &, const unsigned int);
		template<class Archive> 
		friend void ::boost::serialization::load(
			Archive &, matrixop &, const unsigned int);
		template<class Archive> 
		friend void ::boost::serialization::save(
			Archive &, const matrixop &, const unsigned int);
	public:
		matrixop(const std::vector<size_t> &size);
		matrixop(size_t ndims, ...);
		matrixop();
		~matrixop(void);
		matrixop(const matrixop & rhs); // Copy constructor
		//matrixop* clone() const; // Clone creator. Never use.

		matrixop operator + (const matrixop&) const;
		matrixop operator - (const matrixop&) const;
		matrixop operator * (const matrixop&) const;
		matrixop operator % (const matrixop&) const; // Elementwise multiplication
		matrixop operator * (double) const;
		matrixop operator ^ (unsigned int) const;
		bool operator == (const matrixop&) const;
		bool operator != (const matrixop&) const;
		matrixop & operator = (const matrixop&);
		matrixop & operator = (const double*); // Assignment from double array
		bool operator < (const matrixop &) const; // Set ordering

		// TODO: normalize calls to place val first for all set(...)
		void set(const std::vector<size_t> &pos, double val);
		void set(double val, size_t rank, ...);
		void setIndex(double, size_t);
		void setCol(size_t col, const std::vector<double> &data);
		void setCol(size_t col, const double *data);
		void setRow(size_t row, const double *data);
		void setRow(size_t row, const std::vector<double> &data);
		double get(const std::vector<size_t> &pos) const;
		double get(size_t rank, ...) const;
		inline double getIndex(size_t index) const { return _data[index]; }

		void size(std::vector<size_t> &out) const;
		const std::vector<size_t> size() const;
		void print(std::ostream &out = std::cerr) const;
		void writeSV(const std::string &delim = ", ", std::ostream &out = std::cerr, bool givedims = true) const;
		size_t dimensionality() const;
		void clear();
		void resize(const std::vector<size_t> &size);
		void resize(size_t ndims, ...);
		size_t maxSize() const;
		bool issquare() const;
/*
		void squeeze(matrixop &res) const;
		void expand(matrixop &res) const;
		void subset(const std::vector<size_t> &start, const std::vector<size_t> &span, matrixop &res) const;
		void superset(const std::vector<size_t> start, matrixop &res) const;
*/

		double det() const;
		void upperTriangular(matrixop &target, matrixop &secondary) const;
		inline matrixop upperTriangular() { matrixop res(_dims), junk(_dims); upperTriangular(res,junk); return res; }
		void lowerTriangular(matrixop &target, matrixop &secondary) const;
		inline matrixop lowerTriangular() { matrixop res(_dims),junk(_dims); lowerTriangular(res,junk); return res; }
		void transpose(matrixop &target) const;
		inline matrixop transpose() const {matrixop res(_dims); transpose(res); return res; }
		void decompositionQR(matrixop &Q, matrixop &R) const;
		void HouseholderUT(matrixop &target) const;
		inline matrixop HouseholderUT() const { matrixop res(_dims); HouseholderUT(res); return res; }
		void upperHessenberg(matrixop &target) const;
		inline matrixop upperHessenberg() const { matrixop res(_dims); upperHessenberg(res); return res; }
		//void QRalgorithm(matrixop &res, std::vector<double> &evals) const;

		void minors(const std::vector<size_t> &pos, matrixop &res) const;
		inline matrixop minors(const std::vector<size_t> &pos) const { matrixop res(_dims); minors(pos,res); return res; }
		void minors(matrixop &res, size_t rank, ...) const;
		inlinerval(matrixop,minors); // macro to inline a variable length function
		
		void toDoubleArray(double *target) const;
		/*
		template <class T> void to(T &start) const
		{
			std::copy(_data,_data+_datasize, start);
		}
		template <> void to<std::vector<double> >(std::vector<double> &target) const
		{
			target.resize(_datasize);
			std::copy(_data,_data+_datasize,target.begin());
		}
		*/
		void fromDoubleArray(const double *target);
		/*
		template <const class T> void from(const T &start)
		{
			std::copy(start, start+_datasize, _data);
		}
		template <> void from<const std::vector<double> >(const std::vector<double> &target)
		{
			std::copy(target.begin(), target.end(), _data);
		}
		template <class T> void from(T &start)
		{
			std::copy(start, start+_datasize, _data);
		}
		template <> void from<std::vector<double> >(std::vector<double> &target)
		{
			std::copy(target.begin(), target.end(), _data);
		}
		*/
		void inverse(matrixop &res) const;
		inline matrixop inverse() const { matrixop res(_dims); inverse(res); return res; }
		void posFromIndex(size_t index, std::vector<size_t> &pos) const; // duplicate of _getPos!!
		void indexFromPos(size_t &index, std::vector<size_t> pos) const;
	protected:
		// TODO: encapsulate all private and protected members, to allow for IPP providers
		std::vector<size_t> _dims;
	private:
		void _getpos(size_t index, std::vector<size_t> &pos) const;
		void _rowmult(size_t row, double factor);
		void _init(const std::vector<size_t> &size);
		void _realloc(size_t numelems);
		void _free();
		size_t _datasize;
		double *_data;
	public: // Static member functions start here
		// These functions construct a new matrixop. It is NOT in a shared_ptr.
		// Done like this because I then have the choice of pointer container.
		static matrixop* fromDoubleArray(const double *src, size_t rank, ...);
		static matrixop* fromDoubleArray(const double *src, const std::vector<size_t> &size);

		static matrixop diagonal(const std::vector<size_t> &size, double val);
		static matrixop diagonal(double val, size_t rank, ...);
		static matrixop identity(const std::vector<size_t> &size);
		static matrixop identity(size_t rank, ...);
		static inline matrixop fileRead(const char* filename) { return fileRead(filename, -1, -1); }
		static matrixop fileRead(const char* filename, int lineStart, int lineEnd);
	private:
		static void _swaprows(matrixop &source, size_t rowa, size_t rowb);
		static size_t _repivot(matrixop &res, size_t row, bool reverse);
	};

}; // end rtmath

// istream / ostream overrides, used for printing / setting matrices
std::ostream & operator<<(std::ostream &stream, const rtmath::matrixop &ob);
std::istream & operator>>(std::istream &stream, rtmath::matrixop &ob);

// Extend std::less and std::hash to allow for hashing and ordering of *const* matrixops
// If not const, then changes in internal data will break the hashes and ordering.
// TODO: see if ordering may be done based on address in memory.

#undef inlinerval
