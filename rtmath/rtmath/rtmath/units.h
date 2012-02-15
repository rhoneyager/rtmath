#pragma once
/* This namespace provides convenient runtime conversion functions for converting
 * different units. This includes interconversions between prefixes and conversions
 * to different unit types. Complex unit systems may also be introduced, allowing for
 * calculation of conversion factors in equations. The static units are based on the 
 * boost units library, and the other stuff is taken from my head.
 *
 * Used in atmos class for profile reading and interconversion. Used in lbl to ensure 
 * correct dimensionality of functions.
 */
#include <string>
#include <map>
#include <memory>
#include <boost/units/systems/si.hpp>

namespace rtmath {
	namespace units {

		// Class is virtual. May be overridden with classes that do formulaic operations,
		// such as converters to density in ppmv
		class converter
		{
		public:
			converter(const std::string &inUnits, const std::string &outUnits);
			virtual ~converter();
			virtual double convert(double inVal) const;
		protected:
			std::string _inUnits, _outUnits;
		private:
			double _convFactor;
			bool _valid;
			static void _initMap();
			static std::map<std::string, boost::units::quantity<boost::units::si::length> > _unitsLength;
			static std::map<std::string, boost::units::quantity<boost::units::si::pressure> > _unitsPressure;
			static std::map<std::string, boost::units::quantity<boost::units::si::temperature> > _unitsTemp;
		};

	}; // end units
}; // end rtmath

