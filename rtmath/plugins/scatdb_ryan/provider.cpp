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
#include "plugin-scatdb-ryan.h"

namespace rtmath
{
	namespace plugins
	{
		namespace scatdb_ryan
		{

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
						double targetx = i.aeff;
						if (usemd) targetx = i.maxDiamFull;
						// TODO: ADD DB LOADING STATE HERE
						::scatdb_ryan::db::findDB(dbfile);
						if (!dbfile.size())
							RDthrow(::Ryan_Debug::error::xMissingFile())
							<< ::Ryan_Debug::error::otherErrorText("Unable to detect scatdb_ryan database file");
						auto sdb = db::loadDB(dbfile.c_str());

						auto f = filter::generate();
						double freq = rtmath::units::conv_spec("um","GHz").convert(s.wavelength);
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

								c.Qsca = floatLine(0,db::data_entries::CSCA_M)
									+ (((targetx-xval)/(xval2-xval))*floatLine2(0,db::data_entries::CSCA_M));
								c.Qbk = floatLine(0,db::data_entries::CBK_M)
									+ (((targetx-xval)/(xval2-xval))*floatLine2(0,db::data_entries::CBK_M));
								c.g = floatLine(0,db::data_entries::G)
									+ (((targetx-xval)/(xval2-xval))*floatLine2(0,db::data_entries::G));
								c.Qext = floatLine(0,db::data_entries::CEXT_M)
									+ (((targetx-xval)/(xval2-xval))*floatLine2(0,db::data_entries::CEXT_M));
								c.Qabs = floatLine(0,db::data_entries::CABS_M)
									+ (((targetx-xval)/(xval2-xval))*floatLine2(0,db::data_entries::CABS_M));

								c.Qbk /= pi * targetx * targetx;
								c.Qsca /= pi * targetx * targetx;
								c.Qext /= pi * targetx * targetx;
								c.Qabs /= pi * targetx * targetx;
								// TODO: Store state!
								return;
							}
							xval = xval2;
						}

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


