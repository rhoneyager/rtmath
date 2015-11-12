#pragma once
#include "defs.h"
#include <string>
#include <memory>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging_base.h>
#include <Ryan_Debug/registry.h>
namespace rtmath {
	namespace units {
		namespace implementations {
			void DLEXPORT_rtmath_core emit_units_log(const std::string&, ::Ryan_Debug::log::severity_level = ::Ryan_Debug::log::debug_2);
			class Unithandler;
			class converter_provider_registry{};
			class DLEXPORT_rtmath_core Converter_registry_provider
			{
			public:
				Converter_registry_provider();
				~Converter_registry_provider();
				typedef const std::shared_ptr<const Ryan_Debug::registry::options> optsType;
				typedef std::function<bool(optsType)> canConvertType;
				canConvertType canConvert;
				typedef std::function<std::shared_ptr<const Unithandler>(optsType)> constructType;
				constructType constructConverter;
				const char* name;
			};
		}
	}
}
namespace Ryan_Debug {
	namespace registry {
		extern template class usesDLLregistry <
			::rtmath::units::implementations::converter_provider_registry,
			::rtmath::units::implementations::Converter_registry_provider > ;
	}
}
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
		namespace implementations {
			/// Opaque object provided to perform unit manipulations.
			struct DLEXPORT_rtmath_core Unithandler : public Ryan_Debug::registry::handler_external
			{
			protected:
				Unithandler(const char* id);
			public:
				virtual ~Unithandler() {}
				virtual double convert(double input) const = 0;
				virtual bool isValid() const = 0;
				//template <typename T>
				//T convert(T input) const;
			};
		}

		/** \brief Base conversion class
		*
		* Class is virtual. May be overridden with classes that do formulaic operations,
		* such as converters to density in ppmv.
		*
		* Now, with the appropriate DLL loaded, the udunits system will be used for most conversions.
		* The derived classes still have a bit of code for when udunits is not installed.
		**/
		class DLEXPORT_rtmath_core converter :
			virtual public ::Ryan_Debug::registry::usesDLLregistry <
			implementations::converter_provider_registry, implementations::Converter_registry_provider >
		{
		public:
			virtual ~converter();
			virtual double convert(double inVal) const;
			static bool canConvert(const std::string &inUnits, const std::string &outUnits);
			static std::shared_ptr<const implementations::Unithandler> getConverter(
				const std::string &inUnits, const std::string &outUnits);
			//virtual double convertFull
			converter(const std::string &inUnits, const std::string &outUnits);
			bool isValid() const;
		protected:
			converter();
			std::shared_ptr<const implementations::Unithandler> h;
		};

		/// \brief Perform interconversions between frequency, wavelength and wavenumber
		/// (GHz, Hz, m, cm, um, cm^-1, m^-1)
		class DLEXPORT_rtmath_core conv_spec : public converter
		{
		public:
			conv_spec(const std::string &inUnits, const std::string &outUnits);
		};
	}
 }



