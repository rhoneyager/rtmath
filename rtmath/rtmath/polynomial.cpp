#include "Stdafx.h"
#include "polynomial.h"
#include "zeros.h"
#include "quadrature.h"
#include <iostream>
#include <cmath>
#include <vector>

namespace rtmath {
	polynomial::polynomial()
	{
	}

	polynomial::~polynomial()
	{
	}

	polynomial::polynomial(unsigned int pow, double val)
	{
		coeff(pow,val);
	}

	/*
	polynomial::polynomial(polynomial &orig)
	{
		// Nicely copy the old polynomial
		unsigned int max = orig.maxPow();
		for (unsigned int i=0; i<=max; i++)
			_coeffs[i] = orig.coeff(i);
		return;
	}
	*/

	polynomial polynomial::operator+ (polynomial param) const
	{
		polynomial temp;
		// Compare degree of both polynomials and pick max
		unsigned int maxDeg = maxPow();
		if (param.maxPow() > maxDeg) maxDeg = maxPow();
		for (unsigned int i=0;i<=maxPow();i++)
			temp.coeff(i, coeff(i) + param.coeff(i));
		temp.truncate();
		return temp;
	}

	polynomial polynomial::operator+ (double param) const
	{
		polynomial temp;
		temp = *this;
		temp.coeff(0, coeff(0) + param);
		return temp;
	}

	polynomial polynomial::operator- (double param) const
	{
		polynomial temp;
		temp = *this;
		temp.coeff(0, coeff(0) - param);
		return temp;
	}

	polynomial polynomial::operator- (polynomial param) const
	{
		// Easily enough, mult. param by -1 and run add op
		polynomial tempa, tempb;
		tempa = param * -1.0;
		tempb = *this + tempa;
		tempb.truncate();
		return tempb;
	}

	polynomial polynomial::operator* (polynomial param) const
	{
		// This is more complex, unfortunately
		polynomial temp;
		using namespace std;
		for (unsigned int i=0; i<=maxPow(); i++)
			for (unsigned int j=0; j<=param.maxPow(); j++)
			{
				temp.coeff(i+j, temp.coeff(i+j) + (coeff(i) * param.coeff(j)) );
			}
		// Save some memory in case of lots of zeros at beginning
		temp.truncate();
		return temp;
	}

	polynomial polynomial::operator* (double param) const
	{
		// Go through and mult coeffs
		polynomial temp;
		for (unsigned int i=0; i<=maxPow(); i++)
			temp.coeff(i, coeff(i) * param);
		temp.truncate();
		return temp;
	}

	polynomial polynomial::operator^ (unsigned int param) const
	{
		//TODO: use a better method
		polynomial temp;
		temp[0] = 1; // Start the mult.
		for (unsigned int i=1; i<=param; i++)
			temp = temp * (*this);
		return temp;
	}
/*
	polynomial polynomial::operator= (double param)
	{
		// Mildly useless
		polynomial temp;
		temp.coeff(0, param);
		return temp;
	}
*/

	bool polynomial::operator== (double param) const
	{
		if (maxPow() > 0) return false;
		if (coeff(0) == param) return true;
		return false;
	}

	bool polynomial::operator== (polynomial param) const
	{
		// Compare the two polynomials
		// First, check rank
		if (maxPow() != param.maxPow()) return false;
		// Check elementwise
		for (unsigned int i=0;i<=maxPow();i++)
		{
			if (coeff(i) != param.coeff(i)) return false;
		}
		return true;
	}

	bool polynomial::operator!= (polynomial param) const
	{
		if (*this == param) return false;
		return true;
	}

	bool polynomial::operator!= (double param) const
	{
		if (maxPow() > 0) return true;
		if (coeff(0) == param) return false;
		return true;
	}

	double polynomial::eval(double xval) const
	{
		double res = 0.0;
		std::map<unsigned int, double>::const_iterator it;
		for (it = _coeffs.begin(); it != _coeffs.end(); it++)
		{
			if ( (*it).second != 0.0 ) 
			{
				res += (*it).second * pow(xval, static_cast<int>((*it).first) );
			}
		}
		return res;
	}

	double polynomial::operator() (double param) const
	{
		return eval(param);
	}

	/* Should take r-value or be l-value
	double polynomial::operator[] (unsigned int param)
	{
		return coeff(param);
	}
	*/

	double& polynomial::operator[] (const unsigned int param) 
	{
		if (_coeffs.count(param) == 0) _coeffs[param] = 0.0;
		return _coeffs[param];
	}

	void polynomial::erase()
	{
		_coeffs.clear();
	}

	void polynomial::coeff(unsigned int pow, double val)
	{
		_coeffs[pow] = val;
		truncate();
	}

	double polynomial::coeff(unsigned int pow) const
	{
		if (_coeffs.count(pow) == 0) return 0.0;
		// TODO: Fix the need for this diversion
		// Use const_iterator find (res[pow] creates an entry if pow nonexistant)
		std::map<unsigned int, double> res = _coeffs;
		return res[pow];
	}

	polynomial polynomial::deriv(unsigned int pow)
	{
		// TODO: fix calling, as this is such an abuse of the stack
		polynomial res;
		// Take the derivative of a polynomial
		unsigned int maxpow = maxPow();
		unsigned int pires;
		// We will be dropping the least significant #pow terms
		// Perform the differentiation
		for (unsigned int i=0;i<=maxpow-pow;i++)
		{
			pires = 1;
			for (unsigned int j=0;j<=pow-1;j++)
			{
				pires *= i + pow - j;
			}
			res[i] = _coeffs[i+pow] * pires;
		}
		return res;
	}

	void polynomial::zeros(std::set<double> &zpts) const
	{
		throw;
		return;
		// Calculate all zeros through repeated differentiation
		// and good application of Brent's method
		// The first part of the step is to find the areas on 
		// the number line where zeros exist. Then, apply Brent's
		// method on these intervals to find the zeros exactly

		// To begin, find the number of complex zeros that will be expected
		//unsigned int numCzeros = maxPow();
	}
	/*
	void polynomial::zeros(std::set<double> &zpts)
	{
		// Calculate all zeros through repeated differentiation 
		// and solution via Brent's method.
		// Will have up to polynomial degree of zeros
		// Some will not exist, some may be repeated
		std::vector<double> pts;
		unsigned int numzeros = maxPow();
		std::vector<polynomial> derivs;
		polynomial curr;
		unsigned int i;
		for (i=0; i<numzeros;i++)
		{
			curr = deriv(i);
			derivs.push_back(curr);
		}
		// Start at the end. Should have one zero.
		// Zero guaranteed as func. is a polynomial
		std::set<std::set<double> > dzeros;
		std::set<double> proto;
		double zero = 0;
		bool cup = false;
		i++;
		do {
			i--;
			proto.clear();
			// Find the zeros based on those that have gone before
			// Handle start case separately
			if (i==numzeros-1)
			{
				// Eqn is linear. Zero is solvable.
				zero = -1.0 * derivs[i].coeff(0) / derivs[i].coeff(1);
				proto.insert(zero);
				dzeros.insert(proto);
				if (derivs[i].coeff(1) > 0) cup = true;
			} else {
				// Points between crit pts are easier than at ends
				// Solve these first
				// Also, set guarantees that members will be sorted!
				std::set<double>::iterator begin, end;
				begin = dzeros[i+1].begin();
				end = dzeros[i+1].end();
				end--;
				if (dzeros[i+1].size > 1)
				{
					// Need at least two points for in-between ops
					std::set<double>::iterator it, nt;
					for (it = dzeros[i+1].begin(); it != end; it++)
					{
						nt = it;
						nt++;
						// Check that they cross a zero
						if ((*it) * (*nt) > 0) continue;
						// They do, so find it
						zero = rtmath::zeros::findzero(*it,*nt,(void*) derivs[i+1].eval);
						proto.insert(zero);
					}
				}
				// Now to catch those pesky end zeros
				// These are outside of the crit. point bounds
				// Either/or may exist, depending on the behavior
				// of the crit. points / concavity

				// If f < 0 and slope->nearest inf is up, a zero exists
				// If f > 0 and slope->nearest inf is down, a zero exists
				// Otherwise, no
				//TODO: check this

				// Examine the left case first
				if ( (derivs[i].eval(*begin) > derivs[i].eval((*begin) - 1.0) 
							&& derivs[i].eval(*begin) > 0) 
						||
						(derivs[i].eval(*begin) < derivs[i].eval((*begin) - 1.0) 
						 && derivs[i].eval(*begin) < 0 )
						)
				{
					// A zero to the left exists. Find it
					// Point should be within 5 * 1 / (begin slope )
					zero = rtmath::zeros::findzero( (*begin) - 5 * 1 / 
							(derivs[i+1].eval(*begin)),
							*begin, (void*) derivs[i].eval);
					proto.insert(zero);
				}
				// Examine the right case
				if ( (derivs[i].eval(*end) > derivs[i].eval((*end) + 1.0) 
							&& derivs[i].eval(*end) < 0) 
						||
						(derivs[i].eval(*end) < derivs[i].eval((*end) + 1.0) 
						 && derivs[i].eval(*end) > 0 )
						)
				{
					// A zero to the left exists. Find it
					// Point should be within 5 * 1 / (begin slope )
					zero = rtmath::zeros::findzero( (*end) + 5 * 1 / 
							(derivs[i+1].eval(*end)),
							*end, (void*) derivs[i].eval);
					proto.insert(zero);
				}
				// And we're done for this iteration
				dzeros.insert(proto);
			}
		} while (i>0);
		// End of the loop
		// The zeros should be now known!
		pts = dzeros[0];
		return;
	}
	*/
	unsigned int polynomial::maxPow() const
	{
		// Use recursion to find first nonzero exponent
		/*
		unsigned int i=0;
		unsigned int run=0;
		for (;;)
		{
			if (_coeffs.count(i) == 0) break;
			if (_coeffs[i] != 0) run = i;
			i++;
		}
		return run;
		*/
		unsigned int max=0;
		std::map<unsigned int, double>::const_iterator it;
		for (it = _coeffs.begin(); it != _coeffs.end(); it++)
		{
			if ( ( (*it).first > max ) && ( (*it).second != 0.0 ) ) 
				max = (*it).first;
		}
		return max;
	}

	void polynomial::truncate()
	{
		// Get maxPow and truncate past this
		truncate(maxPow() );
	}

	void polynomial::truncate(unsigned int pow)
	{
		// Truncate at the specified power
		unsigned int max = (unsigned int) _coeffs.size();
		for (unsigned int i=pow+1; i<=max; i++)
		{
			// Safety check. If it exists, delete it
			if (_coeffs.count(i)) _coeffs.erase(i);
		}
	}

	void polynomial::print() const
	{
		using namespace std;
//		cerr << "Size: " << _coeffs.size() << endl;
//		cerr << "Pow: " << maxPow() << endl;
		unsigned int max = maxPow();
		for (unsigned int i=max; i>0; i--)
			if (coeff(i) != 0)
			{
				if ( i != max && coeff(i) > 0.0) 
				{
					cerr << "+ ";
				} else {
					if ( coeff(i) < 0.0 )
					{ 
						if (i == max)
						{
							cerr << "-";
						} else {
							cerr << "- ";
						}
					}
				}
				cerr << abs(coeff(i)) << "*x^" << i << " ";
			}
		if (coeff(0) != 0.0) 
		{
			if (coeff(0) > 0.0 && max != 0) cerr << "+ ";
			if (coeff(0) < 0.0)
			{
				if (max != 0)
				{
					cerr << "- ";
				} else {
					cerr << "-";
				}
			}
			cerr << abs(coeff(0));
		}
		cerr << endl;
	}


};

