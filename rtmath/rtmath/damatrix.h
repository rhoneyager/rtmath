#pragma once

// The damatrix is the fundamental unit for doubling-adding calculations

#include <memory>
#include "matrixop.h"

namespace rtmath {
	
	enum daOp
	{
		NONE,
		ADD,
		MULT,
		INV,
		POW
	};

	class damatrix
	{
	public:
		damatrix(const matrixop &source);						// Use matrixop as source
		damatrix(const damatrix &rhs);							// Copy constructor
		virtual ~damatrix();
	private:
		void __init();
	protected:
		damatrix();												// Constructor used for operators
	public:
		//damatrix operator * (damatrix &rhs);					// Multiply two damatrices
		//damatrix operator * (double rhs);						// Convenient alias to multiply by a diagonal matrix
		//damatrix operator + (damatrix &rhs);					// Add two damatrices
		//damatrix operator ^ (unsigned int pow);					// Raise damatrix to a power
		//damatrix inverse();										// Compute the inverse of a damatrix
		virtual std::shared_ptr<matrixop> eval(const mapid &valmap);	// Evaluate the damatrix to a matrixop
	protected:
		std::shared_ptr<matrixop> _provider;
		std::shared_ptr<damatrix> _rootA, _rootB;
		unsigned int _pow;
		daOp _parentOp;
	public: // Static Functions
		// Perform the desired operation involving two damatrices
		static std::shared_ptr<damatrix> op(std::shared_ptr<damatrix> lhs, 
			std::shared_ptr<damatrix> rhs, daOp oper);
	protected: // Static Functions
	};



}; // end namespace rtmath
