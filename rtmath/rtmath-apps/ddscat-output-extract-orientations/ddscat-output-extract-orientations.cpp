/* This program is designed to extract tsv data from sets of ddOutput files. */
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
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
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
		cerr << "rtmath-ddscat-output-extract-orientations\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >(),"specify input directories or files")
			("output,o", po::value<string>(), "specify output file (.tsv)")
			("isotropic-weights,I", "Force weights to be isotropic. Used for nonstandard ddscat sets.")
			("temperature,T", po::value<float>()->default_value(0), "Specify temperature (K)")
			;

		po::positional_options_description p;
		p.add("input",-1);
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
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		else doHelp("Need to specify output file.");
		bool isoWts = false;
		if (vm.count("isotropic-weights")) isoWts = true;

		float temp = vm["temperature"].as<float>();

		ofstream out(sOutput.c_str());
		out << "Filename\tDescription\tShape Hash\tDDSCAT Version Tag\tFrequency (GHz)\tTemperature (K)\t"
			"M_real\tM_imag\tAeff (um)\t"
			"Qsca_m_iso\tQbk_m_iso\tQabs_m_iso\tQext_m_iso\t"
			"Qpha_1_iso\tQpha_2_iso\tQpha_m_iso\tdQpha_iso\t"
			"Beta\tTheta\tPhi\tWeight\t"
			"Qsca_m_ori\tQbk_m_ori\tQabs_m_ori\tQext_m_ori\tG_1_m_ori\t"
			"Qsca_1_ori\tQbk_1_ori\tQabs_1_ori\tQext_1_ori\tG_1_1_ori\t"
			"Qsca_2_ori\tQbk_2_ori\tQabs_2_ori\tQext_2_ori\tG_1_2_ori\t"

			"Qpha_1_ori\tQpha_2_ori\tQpha_m_ori\tdQpha_ori\t"

			"theta_0_phi_0_ReF11\ttheta_0_phi_0_ImF11\ttheta_0_phi_0_ReF21\ttheta_0_phi_0_ImF21\t"
			"theta_0_phi_0_ReF12\ttheta_0_phi_0_ImF12\ttheta_0_phi_0_ReF22\ttheta_0_phi_0_ImF22\t"
			"theta_0_phi_90_ReF11\ttheta_0_phi_90_ImF11\ttheta_0_phi_90_ReF21\ttheta_0_phi_90_ImF21\t"
			"theta_0_phi_90_ReF12\ttheta_0_phi_90_ImF12\ttheta_0_phi_90_ReF22\ttheta_0_phi_90_ImF22\t"

			"theta_180_phi_0_ReF11\ttheta_180_phi_0_ImF11\ttheta_180_phi_0_ReF21\ttheta_180_phi_0_ImF21\t"
			"theta_180_phi_0_ReF12\ttheta_180_phi_0_ImF12\ttheta_180_phi_0_ReF22\ttheta_180_phi_0_ImF22\t"
			"theta_180_phi_90_ReF11\ttheta_180_phi_90_ImF11\ttheta_180_phi_90_ReF21\ttheta_180_phi_90_ImF21\t"
			"theta_180_phi_90_ReF12\ttheta_180_phi_90_ImF12\ttheta_180_phi_90_ReF22\ttheta_180_phi_90_ImF22\t"

			"theta_0_phi_0_ReS11\ttheta_0_phi_0_ImS11\ttheta_0_phi_0_ReS21\ttheta_0_phi_0_ImS21\t"
			"theta_0_phi_0_ReS12\ttheta_0_phi_0_ImS12\ttheta_0_phi_0_ReS22\ttheta_0_phi_0_ImS22\t"
			"theta_0_phi_90_ReS11\ttheta_0_phi_90_ImS11\ttheta_0_phi_90_ReS21\ttheta_0_phi_90_ImS21\t"
			"theta_0_phi_90_ReS12\ttheta_0_phi_90_ImS12\ttheta_0_phi_90_ReS22\ttheta_0_phi_90_ImS22\t"

			"theta_180_phi_0_ReS11\ttheta_180_phi_0_ImS11\ttheta_180_phi_0_ReS21\ttheta_180_phi_0_ImS21\t"
			"theta_180_phi_0_ReS12\ttheta_180_phi_0_ImS12\ttheta_180_phi_0_ReS22\ttheta_180_phi_0_ImS22\t"
			"theta_180_phi_90_ReS11\ttheta_180_phi_90_ImS11\ttheta_180_phi_90_ReS21\ttheta_180_phi_90_ImS21\t"
			"theta_180_phi_90_ReS12\ttheta_180_phi_90_ImS12\ttheta_180_phi_90_ReS22\ttheta_180_phi_90_ImS22\t"

			"t0p0_P11\tt0p0P12\tt0p0P21\tt0p0P22\tt0p0P31\tt0p0P41\t"
			"t0p90_P11\tt0p90P12\tt0p90P21\tt0p90P22\tt0p90P31\tt0p90P41\t"
			"t180p0_P11\tt180p0P12\tt180p0P21\tt180p0P22\tt180p0P31\tt180p0P41\t"
			"t180p90_P11\tt180p90P12\tt180p90P21\tt180p90P22\tt180p90P31\tt180p90P41"
			<< endl;

		using namespace boost::filesystem;
		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				return pf;
			} else {
				return p;
			}
		};

		vector<path> inputs;
		for (const std::string &rin : vsInput)
		{
			path p(rin);
			path ps = expandSymlinks(p);
			inputs.push_back(ps);
			/*
			if (is_directory(ps))
			{
				vector<path> cands;
				copy(recursive_directory_iterator(ps,symlink_option::recurse), 
					recursive_directory_iterator(), back_inserter(inputs));
			} else {
				inputs.push_back(ps);
			}
			*/
		}

		for (const path &p : inputs)
		{
			cerr << "Processing: " << p << endl;
			using namespace rtmath::ddscat;
			boost::shared_ptr<ddOutput> ddOut;
			if (is_directory(p))
			{
				// Input is a ddscat run
				ddOut = ddOutput::generate(p.string());
			} else if (Ryan_Serialization::known_format(p)) {
				// Input may be a ddOutput file
				// Read will fail if it is not the right file type
				ddOut = boost::shared_ptr<ddOutput>(new ddOutput);
				ddOut->readFile(p.string());
			} else {
				cerr << "\tWrong file type\n";
				continue;
			}

			rotations rots;
			ddOut->parfile->getRots(rots);

			double Qbk_iso = 0;
			double Qsca_iso = 0;
			double Qabs_iso = 0;
			double Qext_iso = 0;

			double Qpha_1_iso = 0;
			double Qpha_2_iso = 0;
			double Qpha_m_iso = 0;
			double dQpha_iso = 0;
			if (ddOut->avg)
			{
				Qbk_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QBKM);
				Qsca_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QSCAM);
				Qabs_iso = ddOut->avg->getStatEntry(stat_entries::QABSM);
				Qext_iso = ddOut->avg->getStatEntry(stat_entries::QEXTM);

				Qpha_1_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QPHA1);
				Qpha_2_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QPHA2);
				Qpha_m_iso = ddOut->avg->getStatEntry(stat_entries::QPHAM);
				dQpha_iso = ddOut->avg->getStatEntry(stat_entries::DQPHA);
			}
			using namespace rtmath::ddscat::weights;
			ddWeightsDDSCAT dw(rots);
			OrientationWeights3d::weightTable wts;
			boost::shared_ptr<OrientationWeights3d> ow
				= boost::shared_ptr<OrientationWeights3d> (new DDSCAT3dWeights(dw));

			ow->getWeights(wts);

			
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
				if (!isoWts)
				{
					ot = std::find_if(wts.cbegin(), wts.cend(), 
					[&](const IntervalTable3dEntry &val)
					{
						return sameRots(it->beta(), it->theta(), it->phi(),
							val.at(IntervalTable3dDefs::BETA_MIN), val.at(IntervalTable3dDefs::BETA_MAX), rots.bN(),
							val.at(IntervalTable3dDefs::THETA_MIN), val.at(IntervalTable3dDefs::THETA_MAX), rots.tN(),
							val.at(IntervalTable3dDefs::PHI_MIN), val.at(IntervalTable3dDefs::PHI_MAX), rots.pN());
					});
				}
				double wt = 0;
				if (ot == wts.cend())
				{
					// Only do matching if gridded weights are indicated.
					if (Qbk_iso)
					{
						cerr << "Could not match rotation when calculating weights ("
							<< it->beta() << ", "
							<< it->theta() << ", "
							<< it->phi() << ").\n";
						continue;
					} else {
						if (isoWts)
							wt = 1. / static_cast<double>(ddOut->scas.size());
						else wt = -1.;
					}
				} else wt = ot->at(IntervalTable3dDefs::WEIGHT);

				std::complex<double> m = it->getM();
				double freq = rtmath::units::conv_spec("um","GHz").convert(it->wave());

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

				const double pi = boost::math::constants::pi<double>();

				out << p.string() << "\t" << ddOut->description << "\t"
					<< ddOut->shapeHash.lower << "\t" << ddOut->ddvertag << "\t"
					<< freq << "\t" << temp << "\t" << m.real() << "\t" << m.imag() << "\t"
					<< it->aeff() << "\t"
					<< Qsca_iso << "\t"
					<< Qbk_iso * 4. * pi << "\t"
					<< Qabs_iso << "\t"
					<< Qext_iso << "\t"
					<< Qpha_1_iso << "\t"
					<< Qpha_2_iso << "\t"
					<< Qpha_m_iso << "\t"
					<< dQpha_iso << "\t"

					<< it->beta() << "\t" << it->theta() << "\t" << it->phi() << "\t"
					<< wt << "\t"

					<< it->getStatEntry(stat_entries::QSCAM) << "\t"
					<< it->getStatEntry(stat_entries::QBKM) * 4. * pi << "\t"
					<< it->getStatEntry(stat_entries::QABSM) << "\t"
					<< it->getStatEntry(stat_entries::QEXTM) << "\t"
					<< it->getStatEntry(stat_entries::G1M) << "\t"
					<< it->getStatEntry(stat_entries::QSCA1) << "\t"
					<< it->getStatEntry(stat_entries::QBK1) * 4. * pi << "\t"
					<< it->getStatEntry(stat_entries::QABS1) << "\t"
					<< it->getStatEntry(stat_entries::QEXT1) << "\t"
					<< it->getStatEntry(stat_entries::G11) << "\t"
					<< it->getStatEntry(stat_entries::QSCA2) << "\t"
					<< it->getStatEntry(stat_entries::QBK2) * 4. * pi << "\t"
					<< it->getStatEntry(stat_entries::QABS2) << "\t"
					<< it->getStatEntry(stat_entries::QEXT2) << "\t"
					<< it->getStatEntry(stat_entries::G12) << "\t"

					<< it->getStatEntry(stat_entries::QPHA1) << "\t"
					<< it->getStatEntry(stat_entries::QPHA2) << "\t"
					<< it->getStatEntry(stat_entries::QPHAM) << "\t"
					<< it->getStatEntry(stat_entries::DQPHA) // no tab after this
					;

				ddOutputSingle::scattMatricesContainer &fs = (*fml)->getScattMatrices();
				boost::shared_ptr<const ddScattMatrixF> t0p0, t180p0, t0p90, t180p90;
				for (const auto &s : fs)
				{
					if (s->id() != scattMatrixType::F) continue; // Should never happen
					if (hasSameRot(s->theta(), 0, 0) && hasSameRot(s->phi(), 0, 0) )
						t0p0 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (hasSameRot(s->theta(), 180, 180) && hasSameRot(s->phi(), 0, 0))
						t180p0 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (hasSameRot(s->theta(), 0, 0) && hasSameRot(s->phi(), 90, 90))
						t0p90 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (hasSameRot(s->theta(), 180, 180) && hasSameRot(s->phi(), 90, 90))
						t180p90 = boost::dynamic_pointer_cast<const ddScattMatrixF>(s);
					if (t0p0 && t180p0 && t0p90 && t180p90) break;
				}

				auto writeFType = [](std::ostream &out, const ddScattMatrix::FType &s)
				{
					for (size_t j = 0; j < 2; j++)
					for (size_t i = 0; i < 2; i++)
					{
						// Note the reversed coordinates. This matches ddscat.
						out << "\t" << s(i, j).real();
						out << "\t" << s(i, j).imag();
					}
				};

				writeFType(out, t0p0->getF());
				writeFType(out, t0p90->getF());
				writeFType(out, t180p0->getF());
				writeFType(out, t180p90->getF());

				writeFType(out, t0p0->getS());
				writeFType(out, t0p90->getS());
				writeFType(out, t180p0->getS());
				writeFType(out, t180p90->getS());

				auto writePType = [](std::ostream &out, const ddScattMatrix::PnnType &p, size_t i, size_t j)
				{
					out << "\t" << p(i, j);
				};

				auto writePTypeStd = [&writePType](std::ostream &out, const ddScattMatrix::PnnType &p)
				{
					// Write P11, P12, P21, P22, P31, P41
					writePType(out, p, 0, 0);
					writePType(out, p, 0, 1);
					writePType(out, p, 1, 0);
					writePType(out, p, 1, 1);
					writePType(out, p, 2, 0);
					writePType(out, p, 3, 0);
				};

				writePTypeStd(out, t0p0->mueller());
				writePTypeStd(out, t0p90->mueller());
				writePTypeStd(out, t180p0->mueller());
				writePTypeStd(out, t180p90->mueller());


				out << endl;
				/*

				"theta_0_phi_0_ReF11\ttheta_0_phi_0_ImF11\ttheta_0_phi_0_ReF21\ttheta_0_phi_0_ImF21\t"
				"theta_0_phi_0_ReF12\ttheta_0_phi_0_ImF12\ttheta_0_phi_0_ReF22\ttheta_0_phi_0_ImF22\t"
				"theta_0_phi_90_ReF11\ttheta_0_phi_90_ImF11\ttheta_0_phi_90_ReF21\ttheta_0_phi_90_ImF21\t"
				"theta_0_phi_90_ReF12\ttheta_0_phi_90_ImF12\ttheta_0_phi_90_ReF22\ttheta_0_phi_90_ImF22\t"

				"theta_180_phi_0_ReF11\ttheta_180_phi_0_ImF11\ttheta_180_phi_0_ReF21\ttheta_180_phi_0_ImF21\t"
				"theta_180_phi_0_ReF12\ttheta_180_phi_0_ImF12\ttheta_180_phi_0_ReF22\ttheta_180_phi_0_ImF22\t"
				"theta_180_phi_90_ReF11\ttheta_180_phi_90_ImF11\ttheta_180_phi_90_ReF21\ttheta_180_phi_90_ImF21\t"
				"theta_180_phi_90_ReF12\ttheta_180_phi_90_ImF12\ttheta_180_phi_90_ReF22\ttheta_180_phi_90_ImF22\t"

				"theta_0_phi_0_ReS11\ttheta_0_phi_0_ImS11\ttheta_0_phi_0_ReS21\ttheta_0_phi_0_ImS21\t"
				"theta_0_phi_0_ReS12\ttheta_0_phi_0_ImS12\ttheta_0_phi_0_ReS22\ttheta_0_phi_0_ImS22\t"
				"theta_0_phi_90_ReS11\ttheta_0_phi_90_ImS11\ttheta_0_phi_90_ReS21\ttheta_0_phi_90_ImS21\t"
				"theta_0_phi_90_ReS12\ttheta_0_phi_90_ImS12\ttheta_0_phi_90_ReS22\ttheta_0_phi_90_ImS22\t"

				"theta_180_phi_0_ReS11\ttheta_180_phi_0_ImS11\ttheta_180_phi_0_ReS21\ttheta_180_phi_0_ImS21\t"
				"theta_180_phi_0_ReS12\ttheta_180_phi_0_ImS12\ttheta_180_phi_0_ReS22\ttheta_180_phi_0_ImS22\t"
				"theta_180_phi_90_ReS11\ttheta_180_phi_90_ImS11\ttheta_180_phi_90_ReS21\ttheta_180_phi_90_ImS21\t"
				"theta_180_phi_90_ReS12\ttheta_180_phi_90_ImS12\ttheta_180_phi_90_ReS22\ttheta_180_phi_90_ImS22"

				*/
			}
			std::cerr << "\tProcessed " << ddOut->scas.size() << " orientations.\n";
		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


