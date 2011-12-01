#pragma once
/* macros.h - macro definitions to speed things up
* MSVC 2010 runtimes of rtmath can be slow. File reads are hampered 
* by slow atof calls, and slow parsing of input lines.
*
* Macros here redirect to the standard function or a custom inlined varient, depending on the compiler.
*/

#include <string>
#include <cmath>

namespace rtmath {
	namespace macros {
		inline double m_atof(const char* x)
		{
			double res = 0.0;
			unsigned int remainder = 0;
			unsigned int rembase = 1;
			unsigned int digit = 0;
			// Sign false indicates positive. True is negative
			bool sign = false;
			bool expsign = false;
			unsigned int expi = 0;
			const char* p = x; // Set pointer to beginning of character stream
			bool exponent = false;
			bool decimal = false;
			while (*p != '\0')
			{
				// Do digit checks here (no calls to isdigit)
				// Ignore whitespace
				if (*p == 'e' || *p == 'E') 
				{
					exponent = true;
				} else if (*p == '.') {
					decimal = true;
				} else if (*p == '-') {
					if (!exponent)
					{
						sign = true;
					} else {
						expsign = true;
					}
				} else if (*p == '+') {
					if (!exponent)
					{
						sign = false;
					} else {
						expsign = false;
					}
				} else if (*p == ' ' || *p == '\t') {
					// Ignore whitespace (but disallow endlines)
				} else {
					// It's a digit!
					switch (*p)
					{
					case '0':
						digit = 0;
						break;
					case '1':
						digit = 1;
						break;
					case '2':
						digit = 2;
						break;
					case '3':
						digit = 3;
						break;
					case '4':
						digit = 4;
						break;
					case '5':
						digit = 5;
						break;
					case '6':
						digit = 6;
						break;
					case '7':
						digit = 7;
						break;
					case '8':
						digit = 8;
						break;
					case '9':
						digit = 9;
						break;
					default:
						// Invalid input
						p++;
						continue;
						break;
					}
					// Digit is set. Next, see what to do with it
					if (!decimal && !exponent)
					{
						res *= 10;
						res += digit;
					} else if (decimal && !exponent) {
						remainder *= 10;
						rembase *= 10;
						remainder += digit;
					} else if (exponent) {
						expi *= 10;
						expi += digit;
					}
				}

				p++;
			}
			// Iterated through the string
			// Now, to combine the elements into my double
			res += (double) remainder / (double) rembase;
			if (sign) res *= -1.0;
			if (exponent)
			{
				if (!expsign)
				{
					res *= std::pow(10, (double) expi);
				} else {
					res *= std::pow(10, -1.0 * (double) expi);
				}
			}
			return res;
		}

		inline double m_atof(const std::string &x)
		{
			return m_atof(x.c_str());
		}

		inline int m_atoi(const char *x)
		{
			int res = 0;
			int digit = 0;
			bool sign = false; // false is pos, true is neg
			bool done = false;
			const char* p = x; // Set pointer to beginning of character stream
			while (*p != '\0' && done == false)
			{
				if (*p == '-') {
					sign = true;
				} else if (*p == '+') {
					sign = false;
				} else if (*p == ' ' || *p == '\t') {
					// Ignore whitespace (but disallow endlines)
				} else {
					// It's a digit!
					switch (*p)
					{
					case '0':
						digit = 0;
						break;
					case '1':
						digit = 1;
						break;
					case '2':
						digit = 2;
						break;
					case '3':
						digit = 3;
						break;
					case '4':
						digit = 4;
						break;
					case '5':
						digit = 5;
						break;
					case '6':
						digit = 6;
						break;
					case '7':
						digit = 7;
						break;
					case '8':
						digit = 8;
						break;
					case '9':
						digit = 9;
						break;
					default:
						// Invalid input
						done = true;
						break;
					}
					// Digit is set. Next, see what to do with it
					if (done) break;
					res *= 10;
					res += digit;
				}
				p++;
			}

			// Return the value
			if (sign) res *= -1;
			return res;
		}

		inline int m_atoi(const std::string &x)
		{
			return m_atoi(x.c_str());
		}

		// Borrowed from Ahlquist for use in zero-finding algorithm.
		// Based on slatec.
		inline double pythag(double a, double b)
		{
			// Find sqrt(a^2+b^2) without overflow or underflow
			double p, q, r, t, s;
			if (abs(p) > abs(q))
			{
				p = a;
				q = b;
			} else {
				q = a;
				p = b;
			}

			if (q == 0.0) return p;

			do {
				r = (a/p) * (a/p);
				t = 4.0 + r;
				s = r / t;
				p = p + (2.0 * p * s);
				q = q * s;
			} while (t != 4.0);

			return p;
		}
	};

};


#ifdef _WIN32
#define M_ATOF(x) rtmath::macros::m_atof(x)
#define M_ATOI(x) rtmath::macros::m_atoi(x)
#else
#define M_ATOF(x) atof(x)
#define M_ATOI(x) atoi(x)
#endif
