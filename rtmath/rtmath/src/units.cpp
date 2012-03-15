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

		converter::converter()
		{
			_init("", "");
		}

		void converter::_init(const std::string &inUnits, const std::string &outUnits)
		{
			_inUnits = inUnits;
			_outUnits = outUnits;
			_convFactor = 1;
			_valid = false;
		}

		converter::~converter()
		{
		}

		atmosConv::atmosConv()
		{
		}

		atmosConv::~atmosConv()
		{
		}

		double converter::convert(double inVal) const
		{
			if (_valid) return inVal * _convFactor;
			throw rtmath::debug::xUnimplementedFunction();
			return 0;
		}

		conv_alt::conv_alt(const std::string &inUnits, const std::string &outUnits)
		{
			_init(inUnits,outUnits);
			bool inV = false, outV = false;

			if (inUnits == "cm") { _convFactor *= 0.01; inV = true; }
			if (inUnits == "km") { _convFactor *= 1000.; inV = true; }
			if (inUnits == "m") inV = true;
			if (outUnits == "cm") { _convFactor *= 100.; outV = true; }
			if (outUnits == "km") { _convFactor /= 1000.; outV = true; }
			if (outUnits == "m") outV = true;

			if (inV && outV) _valid = true;
		}

		conv_pres::conv_pres(const std::string &in, const std::string &out)
		{
			_init(in,out);
			bool inV = false, outV = false;

			if (in == "mb" || in == "millibar") { _convFactor *= 100; inV = true; }
			if (in == "hPa") { _convFactor *= 100; inV = true; }
			if (in == "Pa") inV = true;
			if (in == "kPa") { _convFactor *= 1000; inV = true; }
			if (in == "bar") { _convFactor *= 100000; inV = true; }
			if (out == "mb" || out == "millibar" || out == "hPa") { _convFactor /= 100; outV = true; }
			if (out == "bar") { _convFactor /= 100000; outV = true; }
			if (out == "Pa") outV = true;
			if (out == "kPa") { _convFactor /=1000; outV = true;}

			if (inV && outV) _valid = true;
		}

		conv_temp::conv_temp(const std::string &in, const std::string &out)
		{
			_init(in,out);
			bool inV = false, outV = false;
			if (in == "K") inV = true;
			if (out == "K") outV = true;
			if (inV && outV) _valid = true;
		}

		conv_dens::conv_dens(const std::string &in, const std::string &out)
		{
			// Handling only number density here
			// TODO: do other types of conversions
			// Most further stuff requires knowledge of R, thus knowledge of 
			// relative humidity
			_init(in,out);
			bool inV = false, outV = false;
			if (in == "m^-3") inV = true;
			if (in == "cm^-3") {inV = true; _convFactor *= 1e6;}
			if (out == "cm^-3") {outV = true; _convFactor /= 1e6;}
			if (out == "m^-3") outV = true;
			if (inV && outV) _valid = true;
			if (in == "ppmv" && out == "ppmv") _valid = true; // ppmv identity
		}

		conv_spec::conv_spec(const std::string &in, const std::string &out)
		{
			_init(in,out);
			bool inV = false, outV = false;
			_Sin = 1.0; _Sout = 1.0; _Iin = false; _Iout = false;
			const double c = 2.99792458e8;

			// Do natural unit fonversions in Hz
			// Inputs
			if (in == "GHz") { _Sin *= 1.e9; inV = true; }
			if (in == "Hz") { inV = true; }
			if (in == "m") {_Iin = true; inV = true; _Sin /= c; }
			if (in == "cm") {_Iin = true; inV = true; _Sin /= 100; _Sin /= c; }
			if (in == "um") {_Iin = true; inV = true; _Sin /= 1.e6; _Sin /= c; }
			// cm^-1 specifies SPECTRAL WAVENUMBER
			if (in == "cm^-1" || in == "wcm^-1") {_Sin *= c*100; inV = true;}
			if (in == "m^-1" || in == "wm^-1") {_Sin *= c; inV = true;}

			// Outputs
			if (out == "GHz") { _Sout /= 1.e9; outV = true; }
			if (out == "Hz") { outV = true; }
			if (out == "m") { outV = true; _Iout = true; _Sout *= c; }
			if (out == "cm") { outV = true; _Iout = true; _Sout *= 100; _Sout *= c; }
			if (out == "um") { outV = true; _Iout = true; _Sout *= 1.e6; _Sout *= c;}
			if (out == "cm^-1" || out == "wcm^-1") {_Sout /= 100*c; outV = true; }
			if (out == "m^-1" || out == "wm^-1") {_Sout /= c; outV = true; }

			if (inV && outV) _valid = true;
		}

		double conv_spec::convert(double in) const
		{
			// Less trivial set of conversions
			// First, if conversion of prefixes is needed, do it.
			if (!_valid) throw rtmath::debug::xUnimplementedFunction();
			double res = in;
			res *= _Sin;

			// If inversion needed...
			if (_Iin) res = 1.0/res;
			// Res is now in Hz. Do output conversion.
			if (_Iout) res = 1.0/res;
			res *= _Sout;

			return res;
		}

	}; // end namespace units
}; // end rtmath

