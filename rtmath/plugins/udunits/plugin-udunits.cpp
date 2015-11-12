/// \brief Provides Bohren and Huffman pf and cross-section calculations.
#define _SCL_SECURE_NO_WARNINGS

#include <cstdio>
#include <cstdarg>
#include <string>
#include <boost/math/constants/constants.hpp>
#include <udunits2.h>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/debug.h"

#include "plugin-udunits.h"

namespace rtmath
{
	namespace plugins
	{
		namespace udunits
		{
			using std::shared_ptr;
			std::shared_ptr<ut_system> utsys;
			int handle_log(const char* fmt, va_list args) {
				//va_list args;
				//va_start (args, fmt);
				const int sz = 2048;
				char buffer[sz];
				vsnprintf(buffer, sz, fmt, args); // Always null-terminates.
				//va_end (args);
				std::string msg(buffer);
				rtmath::units::implementations::emit_units_log(msg);
			}
			void init() {
				static bool inited = false;
				if (inited) return;
				inited = true;
				ut_set_error_message_handler(handle_log);
				utsys = shared_ptr<ut_system>(ut_read_xml(nullptr), ut_free_system);
				if (!utsys) RDthrow(Ryan_Debug::error::xMissingFile())
					<< Ryan_Debug::error::otherErrorText("Cannot find udunits xml file.");
			}
			using namespace rtmath::units;
			using namespace rtmath::units::implementations;
			bool canConvert(Converter_registry_provider::optsType opts) {
				const std::string in = opts->getVal<std::string>("inUnits");
				const std::string output = opts->getVal<std::string>("outUnits");
				shared_ptr<ut_unit> inunit(
					ut_parse(utsys.get(),in.c_str(),UT_ASCII), ut_free);
				if (!inunit) {
					std::ostringstream out;
					out << "When testing conversion of units " << in << " to " << out
						<< ", the input unit was invalid.";
					emit_units_log(out.str());
					return false; }
				shared_ptr<ut_unit> outunit(
					ut_parse(utsys.get(),output.c_str(),UT_ASCII), ut_free);
				if (!outunit) {
					std::ostringstream out;
					out << "When testing conversion of units " << in << " to " << out
						<< ", the output unit was invalid.";
					emit_units_log(out.str());
					return false; }
				int can_convert = ut_are_convertible(inunit.get(), outunit.get());
				if (!can_convert) { return false; }
				//shared_ptr<cv_converter> converter(ut_get_converter(inunit.get(), outunit.get()), cv_free);
				//if (!converter) { cerr << "Cannot convert create converter" << endl; throw; }
				return true;
			}
			struct udunits_converter : public rtmath::units::implementations::Unithandler {
				udunits_converter(shared_ptr<cv_converter> handle) :
					Unithandler("udunits"), h(handle) {}
				shared_ptr<cv_converter> h;
				virtual ~udunits_converter() {}
				virtual bool isValid() const { if (h) return true; return false; }
				double convert(double inVal) const {
					if (!h) {
						emit_units_log("Calling convert(double) on uninitialized cv_converter!",
							::Ryan_Debug::log::error);
						RDthrow(Ryan_Debug::error::xOtherError())
							<< Ryan_Debug::error::otherErrorText("Calling convert(double) on uninitialized cv_converter!");
						return -1;
					}
					return cv_convert_float(h.get(), inVal);
				}
			};
			std::shared_ptr<const implementations::Unithandler> constructConverter(
					Converter_registry_provider::optsType opts) {
				const std::string in = opts->getVal<std::string>("inUnits");
				const std::string output = opts->getVal<std::string>("outUnits");
				shared_ptr<ut_unit> inunit(
					ut_parse(utsys.get(),in.c_str(),UT_ASCII), ut_free);
				if (!inunit) {
					std::ostringstream out;
					out << "When converting units " << in << " to " << out
						<< ", the input unit was invalid.";
					emit_units_log(out.str());
					return nullptr; }
				shared_ptr<ut_unit> outunit(
					ut_parse(utsys.get(),output.c_str(),UT_ASCII), ut_free);
				if (!outunit) {
					std::ostringstream out;
					out << "When converting units " << in << " to " << out
						<< ", the output unit was invalid.";
					emit_units_log(out.str());
					return nullptr; }
				int can_convert = ut_are_convertible(inunit.get(), outunit.get());
				if (!can_convert) { return nullptr; }
				shared_ptr<cv_converter> converter(ut_get_converter(inunit.get(), outunit.get()), cv_free);
				if (!converter) {
					std::ostringstream out;
					out << "When converting units " << in << " to " << out
						<< ", constructin of the converter failed.";
					emit_units_log(out.str());
					return nullptr;
				}
				//float out = cv_convert_float(converter.get(), input);

				std::shared_ptr<udunits_converter> res(new udunits_converter(converter));
				return res;
			}
		}
	}
}

D_Ryan_Debug_validator();
D_rtmath_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::udunits;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-udunits",
		"Extends unit system to arbitrary units",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	using std::shared_ptr;
	// Initialize udunits sytem - TODO: move to be invoked on first use. Better error handling.
	ut_set_error_message_handler(rtmath::plugins::udunits::handle_log);
	rtmath::plugins::udunits::utsys = shared_ptr<ut_system>(ut_read_xml(nullptr), ut_free_system);
	// Can't throw errors in this part of the code!
	if (!rtmath::plugins::udunits::utsys) { return OTHER_FAILURE; }

	// Register the plugin
	rtmath::units::implementations::Converter_registry_provider r;
	r.canConvert = rtmath::plugins::udunits::canConvert;
	r.constructConverter = rtmath::plugins::udunits::constructConverter;
	static const char* name = "udunits";
	r.name = name;
	Ryan_Debug::registry::doRegisterHook
		<rtmath::units::converter,
		rtmath::units::implementations::converter_provider_registry,
		rtmath::units::implementations::Converter_registry_provider >
		(r);
	return SUCCESS;
}

