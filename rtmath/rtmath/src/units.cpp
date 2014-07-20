#include "Stdafx-core.h"
#include "../rtmath/units.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace units {

		converter::converter()
		{
			_init("", "");
		}

		void converter::_init(const std::string &inUnits, const std::string &outUnits)
		{
			_inOffset = 0;
			_outOffset = 0;
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
			if (_valid) return ((inVal + _inOffset) * _convFactor) + (_outOffset);
			throw rtmath::debug::xBadInput("Trying to convert with bad converter units.");
			return 0;
		}

		conv_alt::conv_alt(const std::string &inUnits, const std::string &outUnits)
		{
			_init(inUnits,outUnits);
			bool inV = false, outV = false;

			if (inUnits == "nm") { _convFactor /= 1e9; inV = true; }
			if (inUnits == "um") { _convFactor /= 1e6; inV = true; }
			if (inUnits == "mm") { _convFactor /= 1e3; inV = true; }
			if (inUnits == "cm") { _convFactor *= 0.01; inV = true; }
			if (inUnits == "km") { _convFactor *= 1000.; inV = true; }
			if (inUnits == "m") inV = true;
			if (outUnits == "nm") { _convFactor *= 1e9; outV = true; }
			if (outUnits == "um") { _convFactor *= 1e6; outV = true; }
			if (outUnits == "mm") { _convFactor *= 1e3; outV = true; }
			if (outUnits == "cm") { _convFactor *= 100.; outV = true; }
			if (outUnits == "km") { _convFactor /= 1000.; outV = true; }
			if (outUnits == "m") outV = true;

			if (inV && outV) _valid = true;
		}

		conv_vol::conv_vol(const std::string &inUnits, const std::string &outUnits)
		{
			_init(inUnits,outUnits);
			bool inV = false, outV = false;

			if (inUnits == "nm^3") { _convFactor /= 1e27; inV = true; }
			if (inUnits == "um^3") { _convFactor /= 1e18; inV = true; }
			if (inUnits == "mm^3") { _convFactor /= 1e9; inV = true; }
			if (inUnits == "cm^3") { _convFactor /= 1e6; inV = true; }
			if (inUnits == "km^3") { _convFactor *= 1e6; inV = true; }
			if (inUnits == "m^3") inV = true;
			if (outUnits == "nm^3") { _convFactor *= 1e27; outV = true; }
			if (outUnits == "um^3") { _convFactor *= 1e18; outV = true; }
			if (outUnits == "mm^3") { _convFactor *= 1e9; outV = true; }
			if (outUnits == "cm^3") { _convFactor *= 1e6; outV = true; }
			if (outUnits == "km^3") { _convFactor /= 1e6; outV = true; }
			if (outUnits == "m^3") outV = true;

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

		conv_mass::	conv_mass(const std::string &in, const std::string &out)
		{
			_init(in,out);
			bool inV = false, outV = false;

			if (in == "ug" ) { _convFactor /= 1e9; inV = true; }
			if (in == "mg") { _convFactor /= 1e6; inV = true; }
			if (in == "g") { _convFactor /= 1e3; inV = true; }
			if (in == "kg") inV = true;
			if (out == "ug") { _convFactor *= 1e9; outV = true; }
			if (out == "mg") { _convFactor *= 1e6; outV = true; }
			if (out == "g") { _convFactor *= 1e3; outV = true;}
			if (out == "kg") outV = true;

			if (inV && outV) _valid = true;
		}

		conv_temp::conv_temp(const std::string &in, const std::string &out)
		{
			_init(in,out);
			bool inV = false, outV = false;
			// K - Kelvin, C - Celsius, F - Fahrenheit, R - Rankine
			if (in == "K") inV = true;
			if (in == "C") { inV = true; _inOffset += 273.15; }
			if (in == "F") { inV = true; _convFactor *= 5./9.; _inOffset += 459.67; }
			if (in == "R") { inV = true; _convFactor *= 5./9; }
			if (out == "K") outV = true;
			if (out == "C") { outV = true; _outOffset -= 273.15; }
			if (out == "F") { outV = true; _convFactor *= 9./5.; _outOffset -= 459.67; }
			if (out == "R") { outV = true; _convFactor *= 9./5.; }
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
			if (in == "mm") { _Iin = true; inV = true; _Sin /= 1.e3; _Sin /= c; }
			// cm^-1 specifies SPECTRAL WAVENUMBER
			if (in == "cm^-1" || in == "wcm^-1") {_Sin *= c*100; inV = true;}
			if (in == "m^-1" || in == "wm^-1") {_Sin *= c; inV = true;}

			// Outputs
			if (out == "GHz") { _Sout /= 1.e9; outV = true; }
			if (out == "Hz") { outV = true; }
			if (out == "m") { outV = true; _Iout = true; _Sout *= c; }
			if (out == "cm") { outV = true; _Iout = true; _Sout *= 100; _Sout *= c; }
			if (out == "um") { outV = true; _Iout = true; _Sout *= 1.e6; _Sout *= c;}
			if (out == "mm") { outV = true; _Iout = true; _Sout *= 1.e3; _Sout *= c; }
			if (out == "cm^-1" || out == "wcm^-1") {_Sout /= 100*c; outV = true; }
			if (out == "m^-1" || out == "wm^-1") {_Sout /= c; outV = true; }

			if (inV && outV) _valid = true;
		}

		double conv_spec::convert(double in) const
		{
			// Less trivial set of conversions
			// First, if conversion of prefixes is needed, do it.
			if (!_valid) RTthrow rtmath::debug::xUnimplementedFunction();
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

