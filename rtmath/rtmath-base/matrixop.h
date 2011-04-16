#pragma once
#include <vector>
#include <map>

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
	virtual ~matrixop(void);
	// Define a copy constructor, since this is a base class
	matrixop (const matrixop & rhs);
	// Cloning function
	virtual matrixop* clone();
	// virtual ...
	virtual matrixop operator + (const matrixop&) const;
	virtual matrixop operator - (const matrixop&) const;
	virtual matrixop operator * (const matrixop&) const;
	virtual matrixop operator * (double) const;
	virtual matrixop operator ^ (unsigned int) const;
	virtual bool operator == (const matrixop&) const;
	virtual bool operator != (const matrixop&) const;
	virtual void set(const std::vector<unsigned int> &pos, double val);
	virtual double get(const std::vector<unsigned int> &pos) const;
	void size(std::vector<unsigned int> &out) const;
	std::vector<unsigned int>& size() const;
	unsigned int dimensionality() const;
	void clear();
	bool issquare() const;
	// To add:
	// Diagonalization
	// Determinant
	// Evaluation
protected:
	std::map<std::vector<unsigned int>, double > _vals;
	std::vector<unsigned int> _dims;
};

}; // end rtmath
