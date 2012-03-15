#pragma once
#include <memory>
#include <cstdarg>
#include <string>
#include <cmath>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "defs.h"
#include "enums.h"
#include "Public_Domain/MurmurHash3.h"

/* coords.h provides the header information for several types of coordinate systems used in rtmath.
   Most importantly, these coordinates are used in ddscat for interpolation of sets of known
   phase matrix results! (see interpolatable.h)
   */

namespace rtmath {
	namespace coords {

		template<class T>
		class cyclic
		{
		protected:
			std::vector<T> _coords;
			size_t _nd;
			friend struct std::less<rtmath::coords::cyclic<T> >;
		public:
			// TODO: fix calls so that the two constructors are distinct!
			cyclic(size_t dims = 2)
			{
				_nd = dims;
				_coords.resize(dims);
			}

			cyclic(size_t ndims, ...)
			{
				_nd = ndims;
				_coords.resize(ndims);
				va_list indices;
				va_start(indices, ndims);
				T ival;
				for (size_t i=0; i<ndims; i++)
				{
					ival = va_arg(indices,T);
					_coords[i] = ival;
				}
				va_end(indices);
			}

			virtual ~cyclic() {}

			virtual T get(size_t index) const
			{
				if (index < _coords.size())
					return _coords[index];
				return 0;
			}

			virtual void get(size_t index, T &ret) const
			{
				if (index < _coords.size())
					ret = _coords[index];
				else
					ret = 0;
			}

			virtual void get(std::vector<T> &res) const
			{
				res = _coords;
			}

			virtual size_t size() const
			{
				return _nd;
			}

			virtual double distance(const cyclic<T> &rhs) const
			{
				// Calculates Eulerian distance in phase space
				// Used heavily in interpolation functions
				// Only give distance is the dimensions are the same!
				if (_nd != rhs._nd) return -1;
				double dsq = 0;
				for (size_t i=0; i<_nd; i++)
				{
					double di = get(i) - rhs.get(i);
					dsq += di * di;
				}
				double d = sqrt(dsq);
				return d;
			}

			virtual HASH_t hash() const
			{
				HASH_t res;
				HASH(this,sizeof(*this),HASHSEED,&res);
				return res;
			}

			bool operator == (const cyclic<T> &rhs) const
			{
				if (_nd != rhs._nd)
					return false;
				
				for (auto it = _coords.begin(), ot = rhs._coords.begin(); it != _coords.end(); ++it,++ot)
					if (*it != *ot) return false;

				return true;
			}

			bool operator != (const cyclic<T> &rhs) const
			{ return !(this->operator==(rhs)); }

			virtual void print(std::ostream &out = std::cerr) const
			{
				using namespace std;
				out << "coords output: ";
				writeCSV(out);
			}

			virtual void writeCSV(std::ostream &out = std::cerr) const
			{
				using namespace std;
				out << _nd << ", ";
				for (auto it = _coords.begin(); it != _coords.end(); ++it)
					out << *it << ", ";
			}

		};

		// Supporting code to allow boost unordered map
		template<class T>
		inline std::size_t hash_value(cyclic<T> const& x)
		{
			return (size_t) x.hash();
		}

		// Hash stuff goes here (for boost unordered_map)

		// Convenient specializations go here
/*
		template<class T>
		class circular : public cyclic<T>
		{
		public:
			circular(T a, T b) : cyclic(2,0,0)
			{
				_coords[0] = a;
				_coords[1] = b;
			}
			//circular() : cyclic(2) {}
			virtual ~circular () {}
		};

		template<class T>
		class spherical : public cyclic<T>
		{
		public:
			spherical(T a, T b, T c) : cyclic(3,0,0,0)
			{
				_coords[0] = a;
				_coords[1] = b;
				_coords[2] = c;
			}
			//spherical() : cyclic(3) {}
			virtual ~spherical() {}
		};
*/
	} // end namespace coords
} // end rtmath

// ostream
template<class T>
std::ostream & operator<<(std::ostream &stream, const rtmath::coords::cyclic<T> &ob)
{
	ob.writeCSV(stream);
	return stream;
}


// Supporting code to allow for unordered_map to work

namespace std {
	template <class T> struct hash<rtmath::coords::cyclic<T> >
	{
		size_t operator()(const rtmath::coords::cyclic<T> & x) const
		{
			// Really need to cast for the unordered map to work
			return (size_t) x.hash();
		}
	};

	template <class T> struct less<rtmath::coords::cyclic<T> >
	{
		bool operator() (const rtmath::coords::cyclic<T> &lhs, const rtmath::coords::cyclic<T> &rhs) const
		{
			// TODO: check that strict weak ordering is preserved
			if (lhs._nd < rhs._nd)
				return true;
			if (lhs._nd > rhs._nd)
				return false;
				
			for (auto it = lhs._coords.begin(), ot = rhs._coords.begin(); it != lhs._coords.end(); ++it,++ot)
			{
				if (*it < *ot) return true;
				if (*it > *ot) return false;
			}
			return false;
		}
	};

} // end namespace std

