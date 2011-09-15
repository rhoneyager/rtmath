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
		matrixop(const std::vector<unsigned int> &size);
		matrixop(unsigned int ndims, ...);
		~matrixop(void);
		// Define a copy constructor, since this is a base class
		matrixop (const matrixop & rhs);
		// Cloning function
		matrixop* clone() const;

		matrixop operator + (const matrixop&) const;
		matrixop operator - (const matrixop&) const;
		matrixop operator * (const matrixop&) const;
		matrixop operator * (double) const;
		matrixop operator ^ (unsigned int) const;
		bool operator == (const matrixop&) const;
		bool operator != (const matrixop&) const;

		void set(const std::vector<unsigned int> &pos, double val);
		void set(double val, unsigned int rank, ...);
		void set(double, unsigned int); // Former _push_back
		double get(const std::vector<unsigned int> &pos) const;
		double get(unsigned int rank, ...) const;
		inline double get(unsigned int index) { return _data[index]; }

		void size(std::vector<unsigned int> &out) const;
		const std::vector<unsigned int> size() const;
		void print() const;
		size_t dimensionality() const;
		void clear();
		void resize(const std::vector<unsigned int> &size);
		void resize(unsigned int ndims, ...);
		unsigned int maxSize() const;
		bool issquare() const;

		double det() const;
		void upperTriangular(matrixop &target) const;
		inline matrixop upperTriangular() { matrixop res(this->_dims); upperTriangular(res); return res; }
		matrixop minors(const std::vector<unsigned int> &pos) const;
        matrixop minors(unsigned int rank, ...) const;
		
		void toDoubleArray(double *target) const;
		void fromDoubleArray(const double *target);
		//void fromCdf(NcVar *var, long n); // Populate the matrixop from a netCDF variable. Specify start location in extended params.
		//void toCDF(NcVar *var) const; // Save matrixop to a netCDF variable. 
		matrixop inverse() const;
		void posFromIndex(size_t index, std::vector<unsigned int> &pos) const; // duplicate of _getPos!!
		void indexFromPos(size_t &index, std::vector<unsigned int> pos) const;
	public:
		static matrixop diagonal(const std::vector<unsigned int> &size, double val);
		static matrixop diagonal(double val, unsigned int rank, ...);
		static matrixop identity(const std::vector<unsigned int> &size);
		static inline matrixop fileRead(const char* filename) { return fileRead(filename, -1, -1); }
		static matrixop fileRead(const char* filename, int lineStart, int lineEnd);
	private:
		static void _swaprows(matrixop &source, size_t rowa, size_t rowb);
		static size_t _repivot(const matrixop &source, matrixop &res, size_t row);
	protected:
		//std::map<std::vector<unsigned int>, double > _vals;
		std::vector<unsigned int> _dims;
	private:
		//void _push_back(unsigned int, double);
		void _getpos(unsigned int index, std::vector<unsigned int> &pos) const;

		// New stuff goes here
	private:
		void _init(const std::vector<unsigned int> &size);
		void _realloc(size_t numelems);
		void _free();
		size_t _datasize;
		double *_data;
	};

}; // end rtmath
