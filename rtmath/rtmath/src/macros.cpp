#include "Stdafx-core.h"
#include "../rtmath/macros.h"

#include <cmath>

namespace rtmath {
	namespace macros {

		template <class T>
		T m_atof(const char* x, size_t len)
		{
			T res = 0;
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
			size_t i=0;
			while (*p != '\0' && ((len)? i<len : true) )
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
				i++;
			}
			// Iterated through the string
			// Now, to combine the elements into my double
			res += (T) remainder / (T) rembase;
			if (sign) res *= -1;
			if (exponent)
			{
				if (!expsign)
				{
					res *= (T) std::pow(10, (T) expi);
				} else {
					res *= (T) std::pow(10, -1.0 * (T) expi);
				}
			}
			return res;
		}

		template <class T>
		T m_atoi(const char *x, size_t len)
		{
			T res = 0;
			int digit = 0;
			bool sign = false; // false is pos, true is neg
			bool done = false;
			size_t i=0;
			const char* p = x; // Set pointer to beginning of character stream
			while (*p != '\0' && done == false && ((len)? i<len : true) )
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
				i++;
			}

			// Return the value
			if (sign) res *= -1;
			return res;
		}

		template double DLEXPORT_rtmath_core m_atof(const char* x, size_t len);
		template float DLEXPORT_rtmath_core m_atof(const char* x, size_t len);

		template int DLEXPORT_rtmath_core m_atoi(const char* x, size_t len);
		//template uint64_t DLEXPORT_rtmath_core m_atoi(const char* x, size_t len);
		template unsigned long long DLEXPORT_rtmath_core m_atoi(const char* x, size_t len);
		template unsigned long DLEXPORT_rtmath_core m_atoi(const char* x, size_t len);
		template long DLEXPORT_rtmath_core m_atoi(const char* x, size_t len);

		/*
		double pythag(double a, double b)
		{
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
		*/

		template<> double fastCast(const std::string &ss)
		{
			return M_ATOF(ss.data());
		}
		template<> float fastCast(const std::string &ss)
		{
			return static_cast<float>(M_ATOF(ss.data()));
		}
		template<> size_t fastCast(const std::string &ss)
		{
			return static_cast<size_t>(M_ATOI(ss.data()));
		}
		template<> int fastCast(const std::string &ss)
		{
			return M_ATOI(ss.data());
		}
	}
}

