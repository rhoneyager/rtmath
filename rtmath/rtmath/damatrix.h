#pragma once
#include "../rtmath-base/matrixop.h"
#include "../rtmath-base/quadrature.h"
// Necessary so that I can ignore the ever-growing damatrix tree
// It can now take care of itself
#include <boost/shared_ptr.hpp>
// Allows variadic function calls
#include <cstdarg>

namespace rtmath {

	class damatrix;

	// TODO: add damatrix operation enums for damatrix::eval to know 
	// what to do with the parents
	enum damatrixopenum
	{
		NONE,
		ADD,
		MULT,
		INV
	};

	/* // Block these while rewriting damatrix
	class daevalmuint : public rtmath::evalfunction
		// Sould really only ever be called from daevalmu
	{
	public:
		daevalmuint(damatrix *targeta, damatrix *targetb);
		damatrix* targeta;
		damatrix* targetb;
		void setA(unsigned int i, unsigned int j,
			double a, double b, double c, unsigned int evalindex);
		void setB(unsigned int i, unsigned int j,
			double a, double b, double c, unsigned int evalindex);
		virtual double eval(double val) const;
		virtual double operator () (double) const;
	private:
		unsigned int _Ai, _Aj, _Bi, _Bj, _An, _Bn;
		double _Aa, _Ab, _Ac, _Ba, _Bb, _Bc;
	};

	class daevalmu : public rtmath::evalfunction
	{
	public:
		daevalmu();
		void setvals(double phi, double phin, double php, 
			double mu, double mun,
			unsigned int i, unsigned int j);
		virtual double eval(double val) const;
		virtual double operator() (double) const;
	private:
		double _phi;
		double _phin;
		double _php;
		double _mu;
		double _mun;
		unsigned int _i, _j;
		damatrix *targeta, *targetb;
	};

	class daevalphi : public rtmath::evalfunction
	{
	public:
		daevalphi();
		void setvals(mapid &valmap);
		void setmats(boost::shared_ptr<damatrix> A, boost::shared_ptr<damatrix> B); 
		// TODO: recode quadrature to perform on matrices
		// evalfunction expects return type of double
		virtual double eval(double val) const;
		virtual double operator() (double) const;
		daevalmu *mueval;
	protected:
		mapid vals;
	};
	*/
	
	// Unfortunately necessary, as an autogenerator could not be 
	// constructed in msvc2010
	struct mmapcomp
	{
		bool operator() (const mapid &lhs, const mapid &rhs)
		{
			if (lhs.phi < rhs.phi) return true;
			if (lhs.phin < rhs.phin) return true;
			if (lhs.mu < rhs.mu) return true;
			if (lhs.mun < rhs.mun) return true;
			return false;
		}
	};

class damatrix :
	public matrixop
	// damatrix is derived from matrixop
	// It implements the standard matrix algebra, except multiplication
	// A product implies matrix mult. and integration over adjoining angles
{
public:
	damatrix(const std::vector<unsigned int> &size) : matrixop(size)
	{
		_provider = NULL;
	}
	// TODO: check to see if I can use variadic args to initialize matrixop
	//damatrix(unsigned int ndims, ...) : matrixop(ndims,...) {}
	// matrixop upconvert structure
	damatrix(const matrixop &source) : matrixop(source.size())
	{
		// The _provider makes a copy of the source
		// This gets too confusing if pointers are used
		_provider = source.clone();
	}
	virtual ~damatrix(void) {}
	damatrix (const damatrix & rhs) : matrixop(rhs.size()) {}; // copy constructor
	virtual damatrix operator * (damatrix&);
	virtual damatrix operator + (damatrix&);
	// eval is introduced in damatrix. providers of initial matrices override this
	virtual boost::shared_ptr<damatrix> eval(const mapid &valmap); // eval at setparams
	// set and get are from matrixop. Disallow their usage for now
	virtual void set(const std::vector<unsigned int> &pos, double val)
	{ throw; }
	virtual void set(double val, unsigned int rank, ...)
	{ throw; }
	virtual double get(const std::vector<unsigned int> &pos) const
	{ throw; return 0; }
	virtual double get(unsigned int rank, ...) const
	{ throw; return 0; }
protected:
	// TODO: use a smart pointer to do a deep copy here
	matrixop *_provider;
	// Must use shared_ptr in all invocations
	boost::shared_ptr<damatrix> _rootA, _rootB;
	damatrixopenum _parentsource;
	std::map<mapid, boost::shared_ptr<damatrix>, mmapcomp > precalc;
private:
	void _precalc_operator();
};

}; // end rtmath
