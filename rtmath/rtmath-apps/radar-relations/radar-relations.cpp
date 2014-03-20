/* This program models ensembles of particles in an atmospheric parcel */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <complex>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/ddweights.h"
#include "../../rtmath/rtmath/units.h"
//#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-radar-relations\n\n";
		const double pi = boost::math::constants::pi<double>();

		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >()->multitoken(), "Input ddscat run directories or files. "
			"Runs are assumed to be at the same frequency and temperature.")
			("output,o", po::value<vector<string> >(), "Specify output files")
			("export-type", po::value<string>(), "Identifier to export (i.e. ar_rot_data)")
			("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")

			("particle-scaling-method,p", po::value<string>()->default_value("uniform"),
			"The method of weighting the orientations (uniform - all particles have the same "
			"rotation profile, ...(TODO)")
			("kappa", po::value<string>()->default_value("0:100:1000:exp"),
			"Kappa (degrees) is comparable to 1/sigma^2. Limit of infinity for only the mean, "
			"and 0 for an isotropic uniform distribution.")
			("mean_theta", po::value<double>()->default_value(0),
			"Theta mean (degrees)")
			("mean_phi", po::value<double>()->default_value(90),
			"Phi mean (degrees)")
			("orientation-weight-method", po::value<string>()->default_value("vMFdual"),
			"Specify the method used in the orientation "
			"calculations (vMF, vMFdual).")

			("observation-angles", po::value<string>()->default_value("0:5:70"),
			"The angles of observation for the particle ensemble (degrees)")

			("size-integration-method", po::value<string>()->default_value("standard"),
			"Method for integrating over particle sizes. standard - integrate from zero to "
			"largest article size, with weights based on a midpoint rule.")
			("size-distribution", po::value<string>()->default_value("uniform"), "Size distribution used in "
			"computing ensemble quantities. (GM: Gunn-Marshall, exp: exponential, uniform")
			("GM_N0", po::value<string>()->default_value("0"), "Gunn-Marshall size distribution N0 values")
			("GM_mu", po::value<string>()->default_value("0"), "Gunn-Marshall mu")
			("GM_lambda", po::value<string>()->default_value("0"), "Gunn-Marshall lambda")
			;

		po::positional_options_description p;
		p.add("input", -1);
		//p.add("output",2);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		Ryan_Serialization::process_static_options(vm);
		rtmath::ddscat::ddUtil::process_static_options(vm);
		//rtmath::ddscat::shapeFileStats::process_static_options(vm);
		rtmath::ddscat::ddOutput::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		vector<string> vsInput;
		string sOutput;
		if (vm.count("input")) vsInput = vm["input"].as<vector<string> >();
		else doHelp("Need to specify input(s)");
		//if (vm.count("output")) sOutput = vm["output"].as<string>();
		//else doHelp("Need to specify output file.");

		double muT = vm["mean_theta"].as<double>();
		double muP = vm["mean_phi"].as<double>();
		string skappas = vm["kappa"].as<string>();
		string sobsangles = vm["observation-angles"].as<string>();

		std::string owmethod = vm["orientation-weight-method"].as<std::string>();
		std::transform(owmethod.begin(), owmethod.end(), owmethod.begin(), ::tolower);

		std::set<double> kappas, obsangles;
		rtmath::config::splitSet(skappas, kappas);
		rtmath::config::splitSet(sobsangles, obsangles);




		using namespace boost::filesystem;
		for (const std::string &rin : vsInput)
		{
			path p(rin);
			p = rtmath::debug::expandSymlink(p);
			cerr << "Processing: " << p << endl;
			using namespace rtmath::ddscat;
			boost::shared_ptr<ddOutput> ddOut;
			if (is_directory(p))
			{
				// Input is a ddscat run
				ddOut = ddOutput::generate(p.string());
			}
			else {
				try {
					// Input may be a ddOutput file
					// Read will fail if it is not the right file type
					ddOut = boost::shared_ptr<ddOutput>(new ddOutput);
					ddOut->readFile(p.string());
				}
				catch (rtmath::debug::xUnknownFileFormat &) {
					cerr << "\tWrong file type\n";
					continue;
				}
			}

			// When file is read, determine a kappa scaling factor based on the shape statistics...
			/// \todo Determine kappa scaling factor based on shape statistics.
			double kappaFactor = 1.;

			/// \todo Determine particle minimum value on a per-particle basis. 
			/// Will eliminate muP and muT from the command-line parameter list.

			rotations rots;
			ddOut->parfile->getRots(rots);

			using namespace rtmath::ddscat::weights;
			ddWeightsDDSCAT dw(rots);
			OrientationWeights3d::weightTable wts;
			using namespace rtmath::ddscat::weights;

			// Iterating over all observation angles and concentration parameters
			for (const auto &kappa : kappas)
				for (const auto &obsAngle : obsangles)
				{


					/// \todo Add a static function to autoselect the orientation weights.
					boost::shared_ptr<OrientationWeights3d> ow;
					if (owmethod == "vmf")
					{
						ow = boost::shared_ptr<OrientationWeights3d>(new VonMisesFisherWeights(
							dw, abs(muT-obsAngle), muP, kappa*kappaFactor)); // Check this
					}
					else if (owmethod == "vmfdual")
					{
						ow = boost::shared_ptr<OrientationWeights3d>(new BimodalVonMisesFisherWeights(
							dw, abs(muT - obsAngle), muP, kappa*kappaFactor)); /// \todo Check this for muT, obsAngle consistency
					}
					else if (owmethod == "iso")
					{
						boost::shared_ptr<OrientationWeights3d> ow
							= boost::shared_ptr<OrientationWeights3d>(new DDSCAT3dWeights(dw));
					}
					else doHelp("Unknown weighting method");

					ow->getWeights(wts);

					double Qbk = 0;
					double Qsca = 0;
					double Qabs = 0;
					double Qext = 0;


					for (const auto &it : ddOut->scas)
					{
						// Convenient function to perform comparisons based on rotation angle
						auto hasSameRot = [](double ang, double amin, double amax) -> bool
						{
							if (ang < amin - 1.e-5) return false;
							if (ang > amax + 1.e-5) return false;
							return true;
						};
						auto sameRots = [&hasSameRot](double beta, double theta, double phi,
							double bmin, double bmax, size_t bn,
							double tmin, double tmax, size_t tn,
							double pmin, double pmax, size_t pn)
						{
							if (bn > 1 && !hasSameRot(beta, bmin, bmax)) return false;
							if (tn > 1 && !hasSameRot(theta, tmin, tmax)) return false;
							if (pn > 1 && !hasSameRot(phi, pmin, pmax)) return false;
							return true;
						};

						// Find the appropriate weight.
						// If not found, just set to the isotropic case weight.
						auto ot = wts.cend();
						ot = std::find_if(wts.cbegin(), wts.cend(),
							[&](const IntervalTable3dEntry &val)
						{
							return sameRots(it->beta(), it->theta(), it->phi(),
								val.at(IntervalTable3dDefs::BETA_MIN), val.at(IntervalTable3dDefs::BETA_MAX), rots.bN(),
								val.at(IntervalTable3dDefs::THETA_MIN), val.at(IntervalTable3dDefs::THETA_MAX), rots.tN(),
								val.at(IntervalTable3dDefs::PHI_MIN), val.at(IntervalTable3dDefs::PHI_MAX), rots.pN());
						});
						double wt = 0;
						if (ot == wts.cend())
						{
							wt = -1.;
						}
						else wt = ot->at(IntervalTable3dDefs::WEIGHT);

						if (wt > 0)
						{
							Qbk += wt * it->getStatEntry(stat_entries::QBKM);
							Qsca += wt * it->getStatEntry(stat_entries::QSCAM);
							Qabs += wt * it->getStatEntry(stat_entries::QABSM);
							Qext += wt * it->getStatEntry(stat_entries::QEXTM);
						}

						std::complex<double> m = it->getM();
						double freq = rtmath::units::conv_spec("um", "GHz").convert(it->wave());

						// Find the fml matrix file matching this entry
						/// \todo Add sca / fml linking as part of ddOutput load
						auto fml = std::find_if(ddOut->fmls.cbegin(), ddOut->fmls.cend(),
							[&](const boost::shared_ptr<ddOutputSingle> &f) -> bool
						{
							// Compare based on rotation angle here
							return sameRots(
								it->beta(), it->theta(), it->phi(),
								f->beta(), f->beta(), 2,
								f->theta(), f->theta(), 2,
								f->phi(), f->phi(), 2);
						});
						if (fml == ddOut->fmls.cend())
						{
							// Should never happen
							cerr << "Could not match rotation when finding fml file ("
								<< it->beta() << ", "
								<< it->theta() << ", "
								<< it->phi() << ").\n";
							continue;
						}


					}

					// The weighted cross-sections have been calculated.
					// Write them.


				}
			std::cerr << "\tProcessed " << ddOut->scas.size() << " orientations.\n";
		}

	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


