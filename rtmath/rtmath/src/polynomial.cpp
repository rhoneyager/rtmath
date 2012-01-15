#include <iostream>
#include <cmath>
#include <vector>
#include <complex>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>


#include "../rtmath/Stdafx.h"
#include "../rtmath/polynomial.h"
#include "../rtmath/matrixop.h"

namespace rtmath {

	void polynomial::_init()
	{
		_var = "x";
	}

	polynomial::polynomial()
	{
		_init();
	}

	polynomial::~polynomial()
	{
	}

	polynomial::polynomial(unsigned int pow, double val)
	{
		_init();
		coeff(pow,val);
	}

	polynomial::polynomial(const polynomial &orig)
	{
		*this = orig;
	}

	polynomial & polynomial::operator=(const polynomial &rhs)
	{
		if (this == &rhs) return *this; // self-assignment check
		_coeffs = rhs._coeffs;
		_var = rhs._var;
		return *this;
	}

	polynomial & polynomial::operator=(double rhs)
	{
		erase();
		coeff(0,rhs);
		return *this;
	}

	polynomial* polynomial::clone() const
	{
		// Cloning operator creates new object
		polynomial *res = new polynomial(*this);
		return res;
	}

	void polynomial::fromDoubleArray(size_t maxdeg, const double* source)
	{
		// Read in data from degree zero upwards
		erase();
		for (size_t i=0; i<=maxdeg; i++)
		{
			coeff(i,source[i]);
		}
	}

	void polynomial::toDoubleArray(double* target) const
	{
		// Start from deg zero and move up to maxPow
		size_t n = maxPow();
		for (size_t i=0; i<=n; i++)
		{
			target[i] = coeff(i);
		}
	}

	polynomial polynomial::operator+ (const polynomial &param) const
	{
		polynomial temp;
		// Compare degree of both polynomials and pick max
		unsigned int maxDeg = maxPow();
		if (param.maxPow() > maxDeg) maxDeg = param.maxPow();
		for (unsigned int i=0;i<=maxDeg;i++)
			temp.coeff(i, coeff(i) + param.coeff(i));
		temp.truncate();
		return temp;
	}

	polynomial& polynomial::operator+= (const polynomial &rhs)
	{
		polynomial a(*this);
		a = a + rhs;
		*this = a;
		return *this;
		/*
		unsigned int maxDeg = maxPow();
		if (rhs.maxPow() > maxDeg) maxDeg = rhs.maxPow();
		for (unsigned int i=0;i<=maxPow();i++)
			coeff(i, coeff(i) + rhs.coeff(i));
		truncate();
		return *this;
		*/
	}

	polynomial& polynomial::operator+= (double rhs)
	{
		coeff(0, coeff(0) + rhs);
		return *this;
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

	polynomial polynomial::operator- (const polynomial &param) const
	{
		polynomial temp;
		// Compare degree of both polynomials and pick max
		unsigned int maxDeg = maxPow();
		if (param.maxPow() > maxDeg) maxDeg = param.maxPow();
		for (unsigned int i=0;i<=maxDeg;i++)
			temp.coeff(i, coeff(i) - param.coeff(i));
		temp.truncate();
		return temp;
	}

	polynomial& polynomial::operator-= (const polynomial &rhs)
	{
		polynomial a(*this);
		a = a - rhs;
		*this = a;
		return *this;
/*		unsigned int maxDeg = maxPow();
		if (rhs.maxPow() > maxDeg) maxDeg = rhs.maxPow();
		for (unsigned int i=0;i<=maxPow();i++)
			coeff(i, coeff(i) - rhs.coeff(i));
		truncate();
		return *this;
		*/
	}

	polynomial& polynomial::operator-= (double rhs)
	{
		coeff(0, coeff(0) - rhs);
		return *this;
	}

	polynomial polynomial::operator* (const polynomial &param) const
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

	polynomial& polynomial::operator*= (const polynomial &rhs)
	{
		polynomial a(*this);
		a = a * rhs;
		*this = a;
		return *this;
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

	polynomial& polynomial::operator*= (double rhs)
	{
		polynomial a = *this;
		a = a * rhs;
		*this = a;
		return *this;
	}

	polynomial polynomial::operator^ (unsigned int param) const
	{
		//TODO: use a better method
		polynomial temp(0,1.0);
		for (unsigned int i=1; i<=param; i++)
			temp = temp * (*this);
		return temp;
	}

	polynomial& polynomial::operator^= (unsigned int rhs)
	{
		polynomial a = *this;
		a = a ^ rhs;
		*this = a;
		return *this;
	}

	bool polynomial::operator== (double param) const
	{
		if (maxPow() > 0) return false;
		if (coeff(0) == param) return true;
		return false;
	}

	bool polynomial::operator== (const polynomial &param) const
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

	bool polynomial::approxEq (double param) const
	{
		if (maxPow() > 0) return false;
		double des = coeff(0) - param;
		des /= coeff(0);
		if (des > 0.0001) return false;
		if (des < -0.0001) return false;
		return true;
	}

	bool polynomial::approxEq (const polynomial &param) const
	{
		// Compare the two polynomials
		// First, check rank
		if (maxPow() != param.maxPow()) return false;
		// Check elementwise
		for (unsigned int i=0;i<=maxPow();i++)
		{
			double des = ( coeff(i) - param.coeff(i)) / coeff(i);
			if (des > 0.000001) return false;
			if (des < -0.000001) return false;
		}
		return true;
	}


	bool polynomial::operator!= (const polynomial &param) const
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

/*
	double& polynomial::operator[] (const unsigned int param) const;
	{
		if (_coeffs.count(param) == 0) _coeffs[param] = 0.0;
		return _coeffs[param];
	}
*/

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

	polynomial polynomial::deriv(unsigned int pow) const
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
			//res[i] = coeff(i+pow) * pires;
			res.coeff(i, coeff(i+pow) * pires);
		}
		return res;
	}

//	void polynomial::zeros(std::set<std::complex<double> > &zpts) const
//	{
//		throw rtmath::debug::xUnimplementedFunction();

		// This work is based on Dr. Ahlquist's 1993 modifications to the 
		// zero-finding functions provided in SLATEC. It computes the complex-
		// valued roots and sorts them. 
		// These roots can be either incorporated into polynomials or treated separately
		// Note: the actual original algorithm allows for complex-coefficient polynomials,
		//  but my class only handles the real-valued case. 
		// TODO: extend this to encompass complex-valued polynomials
		/*
		unsigned int maxpow = maxPow();
		// Consistency check - make sure that the polynomial is not zero
		if (maxpow == 0) return;
		
		// Easy answer: when the polynomial is linear
		if (maxpow == 1)
		{
			double zero = -1.0 * coeff(0) / coeff(1);
			//zpts.insert(std::complex<double>(zero,0));
			return;
		}

		// More complex case that requires actual math
		// Need to set up a scratch area for work calculations
		double *work = new double[2*maxpow*(maxpow+1)];
		using namespace std;
		complex<double> scale;
		complex<double> c;
		complex<double> *coeffs = new complex<double>[maxpow+1];
		complex<double> *roots = new complex<double>[maxpow+1];
		int k, khr, khi, kwr, kwi, kad, kj;

		// TODO: for all of these, check the array bounds, as 
		//  Fortran is annoyingly different than C
		
		// Begin the main loop
		scale = 1.0 / coeff(maxpow);
		khr = 1;
		khi = khr * maxpow * maxpow;
		kwr = khi + khi - khr;
		kwi = kwr + maxpow;
		// Initialize the work space to zero
		//for (k = 0; k < 2 * maxpow * (maxpow + 1); k++)
			//work[k] = 0.0;

		// Next loop
		for (k=0; k < maxpow; k++)
		{
			kad = (k-1) * maxpow + 1;
			c = scale * coeff(k+1);
			work[kad] =  c.real() * -1.0;
			kj = khi + kad - 1;
			work[kj] = -1.0 * c.imag();
			if (k == maxpow) continue;
			work[kad+k] = 1.0;
		}

		// Find the eigenvalues of a complex upper Hessenberg matrix
		// with the QR method


		// Free used memory
		delete[] work;
		delete[] coeffs;
		delete[] roots;
		return;
		// Calculate all zeros through repeated differentiation
		// and good application of Brent's method
		// The first part of the step is to find the areas on 
		// the number line where zeros exist. Then, apply Brent's
		// method on these intervals to find the zeros exactly

		// To begin, find the number of complex zeros that will be expected
		//unsigned int numCzeros = maxPow();
		*/
//	}
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
				cerr << abs(coeff(i)) << "*" << _var << "^" << i << " ";
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


// istream / ostream overrides, used for getting and setting polynomials
std::ostream & operator<<(std::ostream &stream, const rtmath::polynomial &ob)
{
	// Quick and easy output for now
	int n = (int) ob.maxPow();
	stream << "{ ";
	for (int i=n;i>0;i--)
	{
		if (ob.coeff(i) == 0) continue;
		if ( i != n ) stream << " + ";
		stream << ob.coeff(i) << "*x^" << i;
	}
	if (ob.coeff(0)) 
		stream << " + " << ob.coeff(0);
	stream << " }";
	return stream;
}

/*
std::istream & operator>>(std::istream &stream, rtmath::polynomial &ob)
{
	// Use tokenizer to split string
	// String will be split based on + - * ^, variable and spaces
	// It's rather a pain to import this way
	// For example, consider "3.1*x^2 + 2*x + 1"
	// The spacing should first be trimmed for ease of processing
	using namespace boost::algorithm;
	using namespace std;
	//string str = stream.str();
	//erase_all(str, " "); // Get rid of all spaces
	throw; // IMPLEMENT REST OF FUNCTION
}
*/

