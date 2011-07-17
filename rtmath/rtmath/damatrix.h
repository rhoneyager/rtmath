#pragma once
#include "matrixop.h"
#include "quadrature.h"
// Necessary so that I can ignore the ever-growing damatrix tree
// It can now take care of itself
#include "debug.h"
#include <memory>
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
		_parentsource = NONE;
	}
	virtual ~damatrix(void) {}
	virtual damatrix* cloneDa() const;
	damatrix (const damatrix & rhs) : matrixop(rhs.size()) {}; // copy constructor
	virtual damatrix operator * (damatrix&);
	virtual damatrix operator * (double);
	virtual damatrix operator + (damatrix&);
	virtual damatrix inverse();
	// eval is introduced in damatrix. providers of initial matrices override this
	virtual std::shared_ptr<damatrix> eval(const mapid &valmap); // eval at setparams
	// set and get are from matrixop. Disallow their usage for now
	/*
	virtual void set(const std::vector<unsigned int> &pos, double val)
	{ throw; }
	virtual void set(double val, unsigned int rank, ...)
	{ throw; }
	virtual double get(const std::vector<unsigned int> &pos) const
	{ throw; return 0; }
	virtual double get(unsigned int rank, ...) const
	{ throw; return 0; }
	*/
protected:
	// TODO: use a smart pointer to do a deep copy here
	matrixop *_provider;
	// Must use shared_ptr in all invocations
	std::shared_ptr<damatrix> _rootA, _rootB;
	damatrixopenum _parentsource;
	std::map<mapid, std::shared_ptr<damatrix>, mmapcomp > precalc;
private:
	void _precalc_operator();
};

}; // end rtmath
