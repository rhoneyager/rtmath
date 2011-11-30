#pragma once
#include <vector>
#include <map>
#include <cstdarg>
#include <memory>
#include <string>
#include <sstream>
//#include <netcdfcpp.h>
#include "defs.h"
#include "mapid.h"
#include "MurmurHash3.h"

#define inlinerval(x,y) inline x y(size_t rank, ...) const \
	{ x res(_dims); va_list indices; va_start(indices, rank); std::vector<size_t> ptr; \
	size_t ival; for (size_t i=0; i<rank; i++) { ival = va_arg(indices,size_t); \
	ptr.push_back(ival); } va_end(indices); y(ptr,res); return res; }

namespace rtmath {

// Class labeled as defective for now to prevent compilation while base is being changed
	class matrixop
		// Defines a generalized set of tensor operations
		// For now, just use it as a 2d matrix
		//  More dimensions becomes more complex and should
		//  be saved for a more general base class
		// Useful because I can overload this and redefine matrix multiplication
		// This enables arbitrary multiplication / integration
		// It's a base class so I can reuse the code in many different ways
	{
	public:
		matrixop(const std::vector<size_t> &size);
		matrixop(size_t ndims, ...);
		~matrixop(void);
		matrixop(); // Invalid constructor
		// Define a copy constructor, since this is a base class
		matrixop(const matrixop & rhs);
		// Cloning function
		matrixop* clone() const;

		matrixop operator + (const matrixop&) const;
		matrixop operator - (const matrixop&) const;
		matrixop operator * (const matrixop&) const;
		matrixop operator * (double) const;
		matrixop operator ^ (unsigned int) const;
		bool operator == (const matrixop&) const;
		bool operator != (const matrixop&) const;
		matrixop & operator = (const matrixop&);

		void set(const std::vector<size_t> &pos, double val);
		void set(double val, size_t rank, ...);
		void set(double, size_t); // Former _push_back
		void setCol(size_t col, const std::vector<double> &data);
		void setCol(size_t col, const double *data);
		void setRow(size_t row, const double *data);
		void setRow(size_t row, const std::vector<double> &data);
		double get(const std::vector<size_t> &pos) const;
		double get(size_t rank, ...) const;
		inline double get(size_t index) const { return _data[index]; }

		void size(std::vector<size_t> &out) const;
		const std::vector<size_t> size() const;
		void print() const;
		size_t dimensionality() const;
		void clear();
		void resize(const std::vector<size_t> &size);
		void resize(size_t ndims, ...);
		size_t maxSize() const;
		bool issquare() const;

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
		void QRalgorithm(matrixop &res, std::vector<double> &evals) const;

		void minors(const std::vector<size_t> &pos, matrixop &res) const;
		inline matrixop minors(const std::vector<size_t> &pos) const { matrixop res(_dims); minors(pos,res); return res; }
		void minors(matrixop &res, size_t rank, ...) const;
		inlinerval(matrixop,minors); // macro to inline a variable length function
		
		void toDoubleArray(double *target) const;
		void fromDoubleArray(const double *target);
		//void fromCdf(NcVar *var, long n); // Populate the matrixop from a netCDF variable. Specify start location in extended params.
		//void toCDF(NcVar *var) const; // Save matrixop to a netCDF variable. 
		void inverse(matrixop &res) const;
		inline matrixop inverse() const { matrixop res(_dims); inverse(res); return res; }
		void posFromIndex(size_t index, std::vector<size_t> &pos) const; // duplicate of _getPos!!
		void indexFromPos(size_t &index, std::vector<size_t> pos) const;
	public:
		static matrixop diagonal(const std::vector<size_t> &size, double val);
		static matrixop diagonal(double val, size_t rank, ...);
		static matrixop identity(const std::vector<size_t> &size);
		static inline matrixop fileRead(const char* filename) { return fileRead(filename, -1, -1); }
		static matrixop fileRead(const char* filename, int lineStart, int lineEnd);
	private:
		static void _swaprows(matrixop &source, size_t rowa, size_t rowb);
		static size_t _repivot(matrixop &res, size_t row, bool reverse);
	protected:
		//std::map<std::vector<unsigned int>, double > _vals;
		std::vector<size_t> _dims;
	private:
		//void _push_back(unsigned int, double);
		void _getpos(size_t index, std::vector<size_t> &pos) const;
		void _rowmult(size_t row, double factor);

		// New stuff goes here
	private:
		void _init(const std::vector<size_t> &size);
		void _realloc(size_t numelems);
		void _free();
		size_t _datasize;
		double *_data;
	};

}; // end rtmath
