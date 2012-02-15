#include "../rtmath/Stdafx.h"
#include <memory>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <locale>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/units/systems/si.hpp> // Used for atmos readins!
#include "../rtmath/units.h"
#include "../rtmath/error/error.h"

using namespace boost::units;

namespace rtmath {
	namespace units {

		std::map<std::string, quantity<si::length> > converter::_unitsLength;
		std::map<std::string, quantity<si::pressure> > converter::_unitsPressure;
		std::map<std::string, quantity<si::temperature> > converter::_unitsTemp;
		void converter::_initMap()
		{
			_unitsLength["meter"] = 1.0 * si::meters;
			_unitsPressure["pascal"] = 1.0 * si::pascals;
			_unitsPressure["bar"] = 1.0e5 * si::pascals;
			_unitsTemp["Kelvin"] = 1.0 * si::kelvin;
		}

		converter::converter(const std::string &inUnits, const std::string &outUnits)
		{
			if (0) _initMap();
			_inUnits = inUnits;
			_outUnits = outUnits;
			_convFactor = 1;
			_valid = false;

			if (inUnits == "km" && outUnits == "m")
				_convFactor = 1000;
			else if (inUnits == "m" && outUnits == "km")
				_convFactor = 1.e-3;
			else if (inUnits == "Pa" && outUnits == "hPa")
				_convFactor = 0.01;
			else if (inUnits == "hPa" && outUnits == "Pa")
				_convFactor = 100;
			else if (inUnits == "mb" && outUnits == "hPa")
				_convFactor = 1.0;
		}

		converter::~converter()
		{
		}

		double converter::convert(double inVal) const
		{
			if (_valid) return inVal * _convFactor;
			throw rtmath::debug::xUnimplementedFunction();
			return 0;
		}



	}; // end namespace units
}; // end rtmath

