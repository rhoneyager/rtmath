#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <boost/unordered_map.hpp>
#include <iostream>
#include <sstream>
#include "../defs.h"
#include "../Public_Domain/MurmurHash3.h"

namespace rtmath
{
	namespace griddata
	{

		class gridCoords
		{
		public:
			friend struct std::less<rtmath::griddata::gridCoords>;
			gridCoords()
			{
				_lat = 0;
				_lon = 0;
				_level = 0;
				_time = 0;
			}
			gridCoords(double lat, double lon, double level, double time)
			{
				_lat = lat;
				_lon = lon;
				_level = level;
				_time = time;
			}
			~gridCoords() {}
			bool operator == (const gridCoords &rhs) const
			{
				if (this->_lat != rhs._lat) return false;
				if (this->_lon != rhs._lon) return false;
				if (this->_level != rhs._level) return false;
				if (this->_time != rhs._time) return false;
				return true;
			}
			bool operator != (const gridCoords &rhs) const
			{
				return !(this->operator==(rhs));
			}
			inline HASH_t hash() const
			{
				HASH_t res;
				HASH(this, sizeof(*this), HASHSEED, &res);
				return res;
			}
			void print(std::ostream &out = std::cerr) const
			{
				out << "(" << _lat << ", " << _lon << ", " << _level << ", " << _time << ")";
			}
			double lat() const {return _lat;}
			double lon() const {return _lon;}
			double level() const {return _level;}
			double time() const {return _time;}
		private:
			double _lat, _lon, _level, _time;
		};

		// Supporting code to allow boost unordered map
		inline std::size_t hash_value(gridCoords const& x)
		{
			return (size_t) x.hash();
		}

	} // end griddata
} // end rtmath

namespace std {
	template <> struct hash<rtmath::griddata::gridCoords>
	{
		size_t operator()(const rtmath::griddata::gridCoords & x) const
		{
			// Really need to cast for the unordered map to work
			return (size_t) x.hash();
		}
	};


	template <> struct less<rtmath::griddata::gridCoords >
	{
		bool operator() (const rtmath::griddata::gridCoords &lhs, const rtmath::griddata::gridCoords &rhs) const
		{
			// Check f, mu, mun, phi, phin
			if (lhs._lat != rhs._lat) return lhs._lat < rhs._lat;
			if (lhs._lon != rhs._lon) return lhs._lon < rhs._lon;
			if (lhs._level != rhs._level) return lhs._level < rhs._level;
			if (lhs._time != rhs._time) return lhs._time < rhs._time;

			return false;
		}
	};

	std::ostream & operator<<(std::ostream &stream, const rtmath::griddata::gridCoords &ob)
	{
		ob.print(stream);
	}
} // end std
