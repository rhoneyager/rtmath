#pragma once
#include <vector>
#include <map>
#include <cstdarg>

namespace rtmath {

	// Used as an easy structure for mapping and function parameters
	class mapid
	{
	public:
		mapid(double mu, double mun, double phi, double phin)
		{
			this->mu = mu;
			this->mun = mun;
			this->phi = phi;
			this->phin = phin;
		}
		mapid () {}
		virtual bool operator == (const mapid &rhs) const
		{
			if (this->mu != rhs.mu) return false;
			if (this->mun != rhs.mun) return false;
			if (this->phi != rhs.phi) return false;
			if (this->phin != rhs.phin) return false;
			return true;
		}
		virtual bool operator != (const mapid &rhs) const
		{ return !(this->operator==(rhs)); }
		double mu, mun, phi, phin;
	};

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
		unsigned int dimensionality() const;
		void clear();
		virtual void resize(const std::vector<unsigned int> &size);
		virtual void resize(unsigned int ndims, ...);
		bool issquare() const;
		virtual double det() const;
		virtual matrixop minor(unsigned int rank, ...) const;
		virtual matrixop minor(const std::vector<unsigned int> &pos) const;
		// To add:
		// Diagonalization
		// Determinant
		// Evaluation
		void toDoubleArray(double *target);
		void fromDoubleArray(const double *target);
		matrixop inverse() const;
		static matrixop diagonal(const std::vector<unsigned int> &size, double val);
		static matrixop identity(const std::vector<unsigned int> &size);
	protected:
		std::map<std::vector<unsigned int>, double > _vals;
		std::vector<unsigned int> _dims;
	private:
		void _push_back(unsigned int, double);
		void _getpos(unsigned int index, std::vector<unsigned int> &pos) const;
	};

}; // end rtmath
