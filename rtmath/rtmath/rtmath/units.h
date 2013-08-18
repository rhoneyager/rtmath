#pragma once
#include "defs.h"
#include <string>

namespace rtmath {
	/** \brief Provides convenient runtime conversion functions for converting
	 * different units. 
	 *
	 * This includes interconversions between prefixes and conversions
	 * to different unit types. Complex unit systems may also be introduced, allowing for
	 * calculation of conversion factors in equations. The static units are based on the 
	 * boost units library, and the other stuff is taken from my head.
	 *
	 * Used in atmos class for profile reading and interconversion. Used in lbl to ensure 
	 * correct dimensionality of functions.
	 **/
	namespace units {
		typedef std::pair<double, std::string> unit_pair;

		/** \brief Base conversion class
		*
		* Class is virtual. May be overridden with classes that do formulaic operations,
		* such as converters to density in ppmv.
		**/
		class DLEXPORT_rtmath_core converter
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

		/**
		* \brief Useful parent class to allow for conversions that require more than
		* just an input quantity. 
		*
		* For example: conversions between density, 
		* partial pressure and parts per million volumetrically or by mass.
		* It basically just provides another function to conveniently set 
		* these values, and it's up to a derived class to make use of it.
		*
		* \todo Combine some classes.
		* \todo Add angular converters.
		* \todo Add classes w/better dimensional analysis and SI prefix awareness.
		**/
		class DLEXPORT_rtmath_core atmosConv : public converter
		{
		public:
			virtual ~atmosConv();
		protected:
			atmosConv();
			//std::weak_ptr
		};

		/// \brief Linear distance conversions (nm, um, mm, cm, km)
		/// \todo Rename to conv_dist_linear
		class DLEXPORT_rtmath_core conv_alt : public atmosConv
		{
		public:
			conv_alt(const std::string &inUnits, const std::string &outUnits);
		};

		/// Volume conversions (nm^3, um3, cm^3, m^3, km^3)
		class DLEXPORT_rtmath_core conv_vol : public atmosConv
		{
		public:
			conv_vol(const std::string &inUnits, const std::string &outUnits);
		};

		/// Pressure conversions (hPa, kPa, mb, bar)
		class DLEXPORT_rtmath_core conv_pres : public atmosConv
		{
		public:
			conv_pres(const std::string &inUnits, const std::string &outUnits);
		};

		/// Temperature conversions (F, C, K, R)
		class DLEXPORT_rtmath_core conv_temp : public atmosConv
		{
		public:
			conv_temp(const std::string &inUnits, const std::string &outUnits);
		};

		/// Mass conversions (ug, mb, g, kg)
		class DLEXPORT_rtmath_core conv_mass : public atmosConv
		{
		public:
			conv_mass(const std::string &inUnits, const std::string &outUnits);
		};

		/** \brief Density conversions
		*
		* Needs a special invocation of the convert function, as density may
		* be expressed in terms of mass, volume or numerical densities!!!
		* \todo Add other stuff, such as water vapor, to allow for more conversions!
		* \note Currently only supporting numberic density (#/cm^3)
		* \todo Add functionality to handle unit conversions by tokenizing
		**/
		class DLEXPORT_rtmath_core conv_dens : public atmosConv
		{
		public:
			conv_dens(const std::string &inUnits, const std::string &outUnits);
		};

		/// \brief Perform interconversions between frequency, wavelength and wavenumber
		/// (GHz, Hz, m, cm, um, cm^-1, m^-1)
		class DLEXPORT_rtmath_core conv_spec : public converter
		{
		public:
			conv_spec(const std::string &inUnits, const std::string &outUnits);
			virtual double convert(double in) const;
		protected:
			double _Sin, _Sout;
			bool _Iin, _Iout;
		};

	}
 }



