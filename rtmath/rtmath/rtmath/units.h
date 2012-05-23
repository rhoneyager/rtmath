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

		class hasUnits
		{
		public:
			hasUnits(double quant, const std::string &sUnits)
				:
				_units(sUnits),
				_quant(quant)
				{ }
			double quant() const { return _quant; }
			void units(std::string &sUnits) const { sUnits = _units; }
			inline std::string units() const { std::string sUnits; units(sUnits); return sUnits; }
		private:
			std::string _units;
			double _quant;
		};

		// Class is virtual. May be overridden with classes that do formulaic operations,
		// such as converters to density in ppmv
		class converter
		{
		public:
			virtual ~converter();
			virtual double convert(double inVal) const;
			//virtual double convertFull
		protected:
			converter();
			std::string _inUnits, _outUnits;
			double _convFactor;
			double _inOffset, _outOffset;
			bool _valid;
			void _init(const std::string &inUnits, const std::string &outUnits);
		};

		// Useful parent class to allow for conversions that require more than
		// just an input quantity. For example: conversions between density, 
		// partial pressure and parts per million volumetrically or by mass.
		// It basically just provides another function to conveniently set 
		// these values, and it's up to a derived class to make use of it.
		class atmosConv : public converter
		{
		public:
			virtual ~atmosConv();
		protected:
			atmosConv();
			//std::weak_ptr
		};

		class conv_alt : public atmosConv
		{
		public:
			conv_alt(const std::string &inUnits, const std::string &outUnits);
		};

		class conv_pres : public atmosConv
		{
		public:
			conv_pres(const std::string &inUnits, const std::string &outUnits);
		};

		class conv_temp : public atmosConv
		{
		public:
			// Will eventually override convert function
			conv_temp(const std::string &inUnits, const std::string &outUnits);
		};

		class conv_dens : public atmosConv
		{
		public:
			conv_dens(const std::string &inUnits, const std::string &outUnits);
			// Needs a special invocation of the convert function, as density may
			// be expressed in terms of mass, volume or numerical densities!!!
			// TODO: add other stuff, such as water vapor, to allow for more
			// conversions!
			// Currently only supporting numberic density (#/cm^3)
		};

		class conv_spec : public converter
		{
		public:
			// Perform interconversions between frequency, wavelength and wavenumber
			conv_spec(const std::string &inUnits, const std::string &outUnits);
			virtual double convert(double in) const;
		protected:
			double _Sin, _Sout;
			bool _Iin, _Iout;
		};

	}; // end units
}; // end rtmath

