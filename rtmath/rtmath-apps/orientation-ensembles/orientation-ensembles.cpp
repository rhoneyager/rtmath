/* Program is designed to read in a set of ddscat runs, with per-orientation information.
* It then uses one of the runs as an 'anchor', with a fixed concentration parameter. 
* The PE functions of the other flakes are also calculated and compared to the anchor to 
* determine comparable concentration parameters. Each flake's concentration parameters 
* are then determined, and their reweighted cross-sections are then written out.
*/

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <complex>
#include <iostream>
#include <string>
#include <valarray>
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
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-orientation-ensembles\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >()->multitoken(),"specify input directories or files")
			("standard,s", po::value<string>(), "specify the file that is used as the "
			"standard for relative weighting")
			("concentration,k", po::value<string>(), "specify the standard concentration "
			"parameters for weighting")
			("fixed-concentration,f", "Set all flakes to have the standardized concentration parameters")
			("nadir,n", po::value<string>()->default_value("0"), "specify the "
			"nadir angles for observing")
			("output,o", po::value<string>(), "specify output file (.tsv)")
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
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);
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
		string sStd;
		string sStdConcentrations;
		string sNadirs;
		bool fixedConcentration = false;
		if (vm.count("fixed-concentration")) fixedConcentration = true;
		if (vm.count("input")) vsInput = vm["input"].as<vector<string> >();
		else doHelp("Need to specify input(s)");
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		else doHelp("Need to specify output file.");
		if (vm.count("standard")) sStd = vm["standard"].as<string>();
		else if (!fixedConcentration) doHelp("Need to specify standard run.");
		if (vm.count("concentration")) sStdConcentrations = vm["concentration"].as<string>();
		else doHelp("Need to specify standard concentration parameters.");
		sNadirs = vm["nadir"].as<string>();

		rtmath::paramSet<double> concentrations(sStdConcentrations);
		rtmath::paramSet<double> nadirs(sNadirs);

		ofstream out(sOutput.c_str());
		out << "Standard\tStandard Concentration\tNadir Angle\tFilename\tConcentration\tDescription\t"
			"Shape Hash\tDDSCAT Version Tag\tFrequency (GHz)\tM_real\tM_imag\tAeff (um)\t"
			"Qsca_m_iso\tQbk_m_iso\tQabs_m_iso\tQext_m_iso\t"
			"# Angles\t"
			"Qsca_m_ori\tQbk_m_ori\tQabs_m_ori\tQext_m_ori\tG_1_m_ori\t"
			"Qsca_1_ori\tQbk_1_ori\tQabs_1_ori\tQext_1_ori\tG_1_1_ori\t"
			"Qsca_2_ori\tQbk_2_ori\tQabs_2_ori\tQext_2_ori\tG_1_2_ori\t"
			"% Error Qbk\t% Error Qsca"
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

		// Load the standard run
		using namespace rtmath::ddscat;
		using namespace rtmath::ddscat::weights;
		auto ddLoad = [](const boost::filesystem::path &p, boost::shared_ptr<ddOutput> &out)
		{
			if (is_directory(p))
			{
				// Input is a ddscat run
				out = ddOutput::generate(p.string());
			} else if (Ryan_Serialization::known_format(p)) {
				// Input may be a ddOutput file
				// Read will fail if it is not the right file type
				out = boost::shared_ptr<ddOutput>(new ddOutput);
				out->readFile(p.string());
			} else {
				cerr << "\tWrong file type for " << p.string() << "\n";
				throw rtmath::debug::xUnknownFileFormat(p.string().c_str());
			}
		};

		boost::shared_ptr<ddOutput> ddStandard;
		
		if (!fixedConcentration)
		{
			ddLoad(path(sStd), ddStandard);
			// Use the standard run to provide a set of uniform initial weights
			rotations srot;
			ddStandard->parfile->getRots(srot);
			ddWeightsDDSCAT dw(srot);
			// Calculate the moment of inertia / PE function for the initial flake
			// TODO!
			throw rtmath::debug::xUnimplementedFunction();
		}

		// Iterate over the results
		for (const path &p : inputs)
		{
			cerr << "Processing: " << p << endl;

			boost::shared_ptr<ddOutput> ddOut;
			try { 
				ddLoad(p, ddOut);
			} catch (rtmath::debug::xUnknownFileFormat &)
			{ continue; }

			rotations rot;
			ddOut->parfile->getRots(rot);
			ddWeightsDDSCAT dwr(rot);

			// Calculate the moment of inertia / PE function for this flake
			// TODO!
			if (!fixedConcentration)
				throw rtmath::debug::xUnimplementedFunction();

			double Qbk_iso = 0;
			double Qsca_iso = 0;
			double Qabs_iso = 0;
			double Qext_iso = 0;
			if (ddOut->avg)
			{
				Qbk_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QBKM);
				Qsca_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QSCAM);
				Qabs_iso = ddOut->avg->getStatEntry(stat_entries::QABSM);
				Qext_iso = ddOut->avg->getStatEntry(stat_entries::QEXTM);
			}

			// Iterate over all possible standardized concentration parameters
			for (auto stdConc = concentrations.begin(); stdConc != concentrations.end(); ++stdConc)
			{
				// Calculate the concentration parameter (based on the standard)
				// TODO!
				double conc = 0;
				if (!fixedConcentration)
					throw rtmath::debug::xUnimplementedFunction();
				else conc = *stdConc;

				// Iterate over all nadir angles
				for (auto nadir = nadirs.begin(); nadir != nadirs.end(); ++nadir)
				{
					const double muP = 90;
					const double muT = 90 - *nadir;

					// Iterate over the results


					boost::shared_ptr<OrientationWeights3d> ow = 
						boost::shared_ptr<OrientationWeights3d>(new 
						BimodalVonMisesFisherWeights(
						dwr, muT, muP, conc));
					OrientationWeights3d::weightTable wts;
					ow->getWeights(wts);
					
					// Stats table for results after reweighting
					std::valarray<double> stats(stat_entries::NUM_STAT_ENTRIES);


					for (const auto &it : ddOut->scas)
					{
						// Find the appropriate weight.
						// If not found, just set to the isotropic case weight.
						auto ot = wts.cend();
						const bool isoWts = false; // TODO: check to see if this is needed and how
						if (!isoWts)
						{
							ot = std::find_if(wts.cbegin(), wts.cend(), 
								[&](const IntervalTable3dEntry &val)
							{
								// Using a metric approach to avoid linux / windows precision differences
								double metric = 0;
								if (rot.bN() > 1)
								{
									if (it->beta() < val.at(IntervalTable3dDefs::BETA_MIN) - 1.e-5)
										metric += pow(it->beta()-val.at(IntervalTable3dDefs::BETA_MIN),2.);
									if (it->beta() > val.at(IntervalTable3dDefs::BETA_MAX) + 1.e-5)
										metric += pow(it->beta()-val.at(IntervalTable3dDefs::BETA_MAX),2.);
								}
								if (rot.tN() > 1)
								{
									if (it->theta() < val.at(IntervalTable3dDefs::THETA_MIN) - 1.e-5)
										metric += pow(it->theta()-val.at(IntervalTable3dDefs::THETA_MIN),2.);
									if (it->theta() > val.at(IntervalTable3dDefs::THETA_MAX) + 1.e-5)
										metric += pow(it->theta()-val.at(IntervalTable3dDefs::THETA_MAX),2.);
								}
								if (rot.pN() > 1)
								{
									if (it->phi() < val.at(IntervalTable3dDefs::PHI_MIN) - 1.e-5)
										metric += pow(it->phi()-val.at(IntervalTable3dDefs::PHI_MIN),2.);
									if (it->phi() > val.at(IntervalTable3dDefs::PHI_MAX) + 1.e-5)
										metric += pow(it->phi()-val.at(IntervalTable3dDefs::PHI_MAX),2.);
								}
								//cerr << it->beta() << " " << it->theta() << " " << it->phi() << " " << metric << "\n";
								if (metric > 1.e-4) return false;
								return true;
							});
							/* if (ot != wts.cend())
							{
								cerr << "\nMatched " << it->beta() << " " << it->theta() << " " << it->phi() << " ";
							} */
						}
						double wt = 0;
						if (ot == wts.cend())
						{
							// Only do matching if gridded weights are indicated.
							if (Qbk_iso)
							{
								cerr << "Could not match rotation ("
									<< it->beta() << ", "
									<< it->theta() << ", "
									<< it->phi() << ").\n";
								continue;
							} else {
								if (isoWts)
									wt = 1. / static_cast<double>(ddOut->scas.size());
								else wt = -1.;
							}
						} else {
							wt = ot->at(IntervalTable3dDefs::WEIGHT);
							//for (auto a = ot->begin(); a != ot->end(); ++a)
							//	std::cerr << " " << *a;
							//std::cerr << std::endl;
						}

						// Add weighted rotation to the calculated cross-sections
						ddOutputSingle::statTableType is;
						it->getStatTable(is);
						std::valarray<double> s(is.data(), is.size());
						stats += s * wt;
						//cerr << wt << "\t";
					}

					if (ddOut->ms.size() == 0) continue;
					std::complex<double> m = ddOut->ms.at(0);
					/*
					ofstream out(sOutput.c_str());
					out << "Standard\tStandard Concentration\tNadir Angle\tFilename\tConcentration\tDescription\t"
					"Shape Hash\tDDSCAT Version Tag\tFrequency (GHz)\tM_real\tM_imag\tAeff (um)\t"
					"Qsca_m_iso\tQbk_m_iso\tQabs_m_iso\tQext_m_iso\t"
					"# rots\t"
					"Qsca_m_ori\tQbk_m_ori\tQabs_m_ori\tQext_m_ori\tG_1_m_ori\t"
					"Qsca_1_ori\tQbk_1_ori\tQabs_1_ori\tQext_1_ori\tG_1_1_ori\t"
					"Qsca_2_ori\tQbk_2_ori\tQabs_2_ori\tQext_2_ori\tG_1_2_ori\t"
					"\% Error Qbk\t\% Error Qsca"
					<< endl;
					*/
					out << sStd << "\t" << *stdConc << "\t" << *nadir << "\t"
						<< p.string() << "\t" << conc << "\t" << ddOut->description << "\t"
						<< ddOut->shapeHash.lower << "\t" << ddOut->ddvertag << "\t"
						<< ddOut->freq << "\t" << m.real() << "\t" << m.imag() << "\t"
						<< ddOut->aeff << "\t"
						<< Qsca_iso << "\t"
						<< Qbk_iso << "\t"
						<< Qabs_iso << "\t"
						<< Qext_iso << "\t"
						<< ddOut->scas.size() << "\t"
						<< stats[stat_entries::QSCAM] << "\t"
						<< stats[stat_entries::QBKM] << "\t"
						<< stats[stat_entries::QABSM] << "\t"
						<< stats[stat_entries::QEXTM] << "\t"
						<< stats[stat_entries::G1M] << "\t"
						<< stats[stat_entries::QSCA1] << "\t"
						<< stats[stat_entries::QBK1] << "\t"
						<< stats[stat_entries::QABS1] << "\t"
						<< stats[stat_entries::QEXT1] << "\t"
						<< stats[stat_entries::G11] << "\t"
						<< stats[stat_entries::QSCA2] << "\t"
						<< stats[stat_entries::QBK2] << "\t"
						<< stats[stat_entries::QABS2] << "\t"
						<< stats[stat_entries::QEXT2] << "\t"
						<< stats[stat_entries::G12] << "\t"
						<< 100. * (stats[stat_entries::QBKM]- Qbk_iso)/ Qbk_iso << "\t"
						<< 100. * (stats[stat_entries::QSCAM] - Qsca_iso)/ Qsca_iso << "\n";
						;
					std::cerr << "\tProcessed conc " << conc << " nadir " << *nadir << " with " << ddOut->scas.size() << " orientations.\n";
				}
			}
		}


	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


