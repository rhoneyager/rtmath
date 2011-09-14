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
		virtual ~matrixop(void);
		// Define a copy constructor, since this is a base class
		matrixop (const matrixop & rhs);
		// Cloning function
		virtual matrixop* clone() const;
		// virtual ...
		virtual matrixop operator + (const matrixop&) const;
		virtual matrixop operator - (const matrixop&) const;
		virtual matrixop operator * (const matrixop&) const;
		virtual matrixop operator * (double) const;
		virtual matrixop operator ^ (unsigned int) const;
		virtual bool operator == (const matrixop&) const;
		virtual bool operator != (const matrixop&) const;
		virtual void set(const std::vector<unsigned int> &pos, double val);
		virtual void set(double val, unsigned int rank, ...);
		virtual double get(const std::vector<unsigned int> &pos) const;
		virtual double get(unsigned int rank, ...) const;
		void size(std::vector<unsigned int> &out) const;
		const std::vector<unsigned int> size() const;
		virtual void print() const;
		unsigned int dimensionality() const;
		void clear();
		virtual void resize(const std::vector<unsigned int> &size);
		virtual void resize(unsigned int ndims, ...);
		unsigned int maxSize() const;
		bool issquare() const;
		virtual double det() const;
		virtual matrixop minors(const std::vector<unsigned int> &pos) const;
                virtual matrixop minors(unsigned int rank, ...) const;
		
		void toDoubleArray(double *target) const;
		void fromDoubleArray(const double *target);
		//void fromCdf(NcVar *var, long n); // Populate the matrixop from a netCDF variable. Specify start location in extended params.
		//void toCDF(NcVar *var) const; // Save matrixop to a netCDF variable. 
		matrixop inverse() const;
		void posFromIndex(unsigned int index, std::vector<unsigned int> pos) const;
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
		std::map<std::vector<unsigned int>, double > _vals;
		std::vector<unsigned int> _dims;
	private:
		void _push_back(unsigned int, double);
		void _getpos(unsigned int index, std::vector<unsigned int> &pos) const;
	};

}; // end rtmath
