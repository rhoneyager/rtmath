/* This program is designed to extract tsv data from sets of ddOutput files. */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
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

		ofstream out(sOutput.c_str());
		out << "Filename\tDescription\tShape Hash\tDDSCAT Version Tag\tFrequency (GHz)\t"
			"M_real\tM_imag\tAeff (um)\t"
			"Qsca_m_iso\tQbk_m_iso\tQabs_m_iso\tQext_m_iso\t"
			"Beta\tTheta\tPhi\tWeight\t"
			"Qsca_m_ori\tQbk_m_ori\tQabs_m_ori\tQext_m_ori\t"
			"Qsca_1_ori\tQbk_1_ori\tQabs_1_ori\tQext_1_ori\t"
			"Qsca_2_ori\tQbk_2_ori\tQabs_2_ori\tQext_2_ori"
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
			if (ddOut->avg)
			{
				Qbk_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QBKM);
				Qsca_iso = ddOut->avg->getStatEntry(rtmath::ddscat::stat_entries::QSCAM);
				Qabs_iso = ddOut->avg->getStatEntry(stat_entries::QABSM);
				Qext_iso = ddOut->avg->getStatEntry(stat_entries::QEXTM);
			}
			using namespace rtmath::ddscat::weights;
			ddWeightsDDSCAT dw(rots);
			OrientationWeights3d::weightTable wts;
			boost::shared_ptr<OrientationWeights3d> ow
				= boost::shared_ptr<OrientationWeights3d> (new DDSCAT3dWeights(dw));

			ow->getWeights(wts);


			for (const auto &it : ddOut->scas)
			{
				// Find the appropriate weight.
				// If not found, just set to the isotropic case weight.
				auto ot = wts.cend();
				if (!isoWts)
				{
					ot = std::find_if(wts.cbegin(), wts.cend(), 
					[&](const IntervalTable3dEntry &val)
					{
						if (rots.bN() > 1)
						{
							if (it->beta() < val.at(IntervalTable3dDefs::BETA_MIN) - 1.e-5) return false;
							if (it->beta() > val.at(IntervalTable3dDefs::BETA_MAX) + 1.e-5) return false;
						}
						if (rots.tN() > 1)
						{
							if (it->theta() < val.at(IntervalTable3dDefs::THETA_MIN) - 1.e-5) return false;
							if (it->theta() > val.at(IntervalTable3dDefs::THETA_MAX) + 1.e-5) return false;
						}
						if (rots.pN() > 1)
						{
							if (it->phi() < val.at(IntervalTable3dDefs::PHI_MIN) - 1.e-5) return false;
							if (it->phi() > val.at(IntervalTable3dDefs::PHI_MAX) + 1.e-5) return false;
						}
						return true;
					});
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
				} else wt = ot->at(IntervalTable3dDefs::WEIGHT);

				/* out << "Filename\tDescription\tShape Hash\tDDSCAT Version Tag\tFrequency (GHz)\t"
				"M_real\tM_imag\tAeff (um)\t"
				"Qsca_iso\tQbk_iso\tQabs_iso\tQext_iso"
				"Beta\tTheta\tPhi\tWeight\tQsca_ori\tQbk_ori\tQabs_ori\tQext_ori"<< endl;
				*/
				std::complex<double> m = it->getM();
				double freq = rtmath::units::conv_spec("um","GHz").convert(it->wave());
				out << p.string() << "\t" << ddOut->description << "\t"
					<< ddOut->shapeHash.lower << "\t" << ddOut->ddvertag << "\t"
					<< freq << "\t" << m.real() << "\t" << m.imag() << "\t"
					<< it->aeff() << "\t"
					<< Qsca_iso << "\t"
					<< Qbk_iso << "\t"
					<< Qabs_iso << "\t"
					<< Qext_iso << "\t"
					<< it->beta() << "\t" << it->theta() << "\t" << it->phi() << "\t"
					<< wt << "\t"
					<< it->getStatEntry(stat_entries::QSCAM) << "\t"
					<< it->getStatEntry(stat_entries::QBKM) << "\t"
					<< it->getStatEntry(stat_entries::QABSM) << "\t"
					<< it->getStatEntry(stat_entries::QEXTM) << "\t"
					<< it->getStatEntry(stat_entries::G1M) << "\t"
					<< it->getStatEntry(stat_entries::QSCA1) << "\t"
					<< it->getStatEntry(stat_entries::QBK1) << "\t"
					<< it->getStatEntry(stat_entries::QABS1) << "\t"
					<< it->getStatEntry(stat_entries::QEXT1) << "\t"
					<< it->getStatEntry(stat_entries::G11) << "\t"
					<< it->getStatEntry(stat_entries::QSCA2) << "\t"
					<< it->getStatEntry(stat_entries::QBK2) << "\t"
					<< it->getStatEntry(stat_entries::QABS2) << "\t"
					<< it->getStatEntry(stat_entries::QEXT2) << "\t"
					<< it->getStatEntry(stat_entries::G12) << "\n"
					;
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


