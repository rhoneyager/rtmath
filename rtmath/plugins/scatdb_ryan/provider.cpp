/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <boost/math/constants/constants.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/conversions/convertLength.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/units.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>
#include <scatdb_ryan/scatdb_ryan.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <Ryan_Debug/logging.h>
#include "plugin-scatdb-ryan.h"


#undef mylog
#undef FL
//#define FL __FILE__ << ", " << (int)__LINE__ << ": "
#define FL "scatdb_ryan/provider.cpp, line " << (int)__LINE__ << ": "

#define mylog(x) { std::ostringstream l; l << FL << x; rtmath::plugins::scatdb_ryan::implementations::emit_log(l.str(), Ryan_Debug::log::warning); }

namespace rtmath
{
	namespace plugins
	{
		namespace scatdb_ryan
		{

			namespace implementations {
				BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
					m_slog,
					boost::log::sources::severity_channel_logger_mt< >,
					(boost::log::keywords::severity = Ryan_Debug::log::error)
					(boost::log::keywords::channel = "plugin_scatdb_ryan"));

				void emit_log(const std::string &m, ::Ryan_Debug::log::severity_level sev)
				{
					auto& lg = rtmath::plugins::scatdb_ryan::implementations::m_slog::get();
					BOOST_LOG_SEV(lg, sev) << m;
				}
			}

			namespace main {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
				{
					using namespace ::rtmath::phaseFuncs;
					using std::string;
					const double pi = boost::math::constants::pi<double>();

					// Desired effective radius is i.aeff
					// Desired max dimension is i.maxDiamFull
					// Desired refractive index is i.m - UNUSED
					// Desired aspect ratio is i.eps
					// Desired flake type is in i.other->getVal<int>("flaketype")
					// Flag for interpolation vs nearest match is bool "interpolate"
					// Flag for aeff vs max dimension is bool "usemd"
					// Temperature is ignored for now
					// Wavelength is s.wavelength
					// i.vFrac is ignored
					// Repeated querying of the database would be rather slow. So, state is supported.
					// The output cross sections has an opaque member that can be re-fed into the
					// setup registry.
					try {
						using namespace ::scatdb_ryan;
						string dbfile = i.other->getVal<string>("dbfile", "");
						bool usemd = i.other->getVal<bool>("usemd", false);
						bool doInterp = i.other->getVal<bool>("interp", true);
						bool doLowess = i.other->getVal<bool>("lowess", true);
						string flaketypes = i.other->getVal<string>("flaketypes", "20");
						string aeffrange = i.other->getVal<string>("aeffrange", "");
						string mdrange = i.other->getVal<string>("mdrange", "");
						string arrange = i.other->getVal<string>("arrange","");
						auto cl = rtmath::units::converter::getConverter(i.lengthUnits, "m");
						auto cs = rtmath::units::converter::getConverter(s.lengthUnits, "m");
						double targetx = cl->convert(i.aeff) * 1e6; // in um
						if (usemd) targetx = cl->convert(i.maxDiamFull) * 1000; // in mm
						double freq = rtmath::units::conv_spec("m","GHz").convert(cs->convert(s.wavelength));
						mylog("scatdb_ryan input requested for:\n"
							"\tdbfile: " << dbfile << "\n\tusemd: " << usemd
							<< "\n\tinterp: " << doInterp << "\n\tlowess: " << doLowess
							<< "\n\tflaketypes: " << flaketypes << "\n\taeffrange: " << aeffrange
							<< "\n\tmdrange: " << mdrange << "\n\tarrange: " << arrange
							<< "\n\ttargetx: " << targetx << "\n\tfreq: " << freq);
						// TODO: ADD DB LOADING STATE HERE
						::scatdb_ryan::db::findDB(dbfile);
						if (!dbfile.size())
							RDthrow(::Ryan_Debug::error::xMissingFile())
							<< ::Ryan_Debug::error::otherErrorText("Unable to detect scatdb_ryan database file");
						mylog("Loading scatdb_ryan file: " << dbfile);
						auto sdb = db::loadDB(dbfile.c_str());

						auto f = filter::generate();

						f->addFilterFloat(db::data_entries::FREQUENCY_GHZ, freq-0.01, freq+0.01);
						f->addFilterInt(db::data_entries::FLAKETYPE, flaketypes);
						//if (vm.count("temp")) f->addFilterFloat(db::data_entries::TEMPERATURE_K, vm["temp"].as<string>());
						if (aeffrange.size()) f->addFilterFloat(db::data_entries::AEFF_UM, aeffrange);
						if (mdrange.size()) f->addFilterFloat(db::data_entries::MAX_DIMENSION_MM, mdrange);
						if (arrange.size()) f->addFilterFloat(db::data_entries::AS_XY, arrange);
						// TODO: ADD FILTER STATE COMPARISON HERE

						auto sdb_filtered = f->apply(sdb);
						db::data_entries::data_entries_floats xaxis = db::data_entries::AEFF_UM;
						if (usemd) xaxis = db::data_entries::MAX_DIMENSION_MM;

						auto s_sorted = sdb_filtered->sort(xaxis);

						auto le_filtered = s_sorted;
						if (doLowess) {
							le_filtered = s_sorted->regress(xaxis);
						}
						auto res = le_filtered;
						if (doInterp) {
							res = le_filtered->interpolate(xaxis);
						}
						// Take results and so a linear interpolation between the two
						// closest target points.
						int numLines = res->floatMat.rows();
						mylog("Final table has " << numLines << " rows.");
						double xval = 0, xval2 = 0;
						for(size_t i=0; i<numLines; ++i) {
							auto floatLine = res->floatMat.block<1,db::data_entries::NUM_DATA_ENTRIES_FLOATS>(std::min(i-1,i),0);
							auto floatLine2 = res->floatMat.block<1,db::data_entries::NUM_DATA_ENTRIES_FLOATS>(i,0);

							// Range check
							double xval2 = 0;
							if (!usemd)
								xval2 = floatLine2(0,db::data_entries::AEFF_UM);
							else
								xval2 = floatLine2(0,db::data_entries::MAX_DIMENSION_MM);
							if (xval2 > targetx) {
								double dx = targetx - xval;

								auto dinterp = [&](int quant) {
									double y1 = floatLine(0,quant);
									double y2 = floatLine2(0,quant);
									double m = (y2-y1)/(xval2-xval);
									double res = y1 + (m*dx);
									if (y1 < 0 || y2 < 0) res = -1;
									return res;
								};
								c.Csca = dinterp(db::data_entries::CSCA_M);
								c.Cext = dinterp(db::data_entries::CEXT_M);
								c.Cbk = dinterp(db::data_entries::CBK_M);
								c.Cabs = dinterp(db::data_entries::CABS_M);
								c.g = dinterp(db::data_entries::G);

								mylog("Found result with xval " << xval << " and xval2 " << xval2
									<< " for target " << targetx
									<< "\n\tCbk1: " << floatLine(0,db::data_entries::CBK_M)
									<< "\n\tCbk2: " << floatLine2(0,db::data_entries::CBK_M)
									<< "\n\tCbk_res: " << c.Cbk);

								// TODO: Store state!
								return;
							}
							xval = xval2;
						}
						c.Csca = -1;
						c.g = -1;
						c.Cbk = -1;
						c.Cext = -1;
						c.Cabs = -1;
						mylog("Out of range. No data returned.");
					}
					catch (std::exception &e)
					{
						std::cerr << "A scatdb_ryan error has occurred!" << std::endl;
						throw e;
					}
					catch (...) {
						//std::cerr << "\t" << t.what() << std::endl;
						RDthrow(Ryan_Debug::error::xOtherError())
							<< Ryan_Debug::error::otherErrorText("A scatdb_ryan error has occurred");
					}
				}
			}
		}
	}
}


