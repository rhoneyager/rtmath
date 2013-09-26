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

		ofstream out(sOutput.c_str());
		out << "Filename\tDescription\tShape Hash\tDDSCAT Version Tag\tFrequency (GHz)\t"
			"M_real\tM_imag\tAeff (um)\t"
			"Qsca_iso\tQbk_iso\tQabs_iso\tQext_iso\t"
			"Beta\tTheta\tPhi\tWeight\tQsca_ori\tQbk_ori\tQabs_ori\tQext_ori"<< endl;

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

			
			for (auto it = wts.cbegin(); it != wts.cend(); ++it)
			{
				// Find the appropriate sca file
				/// \todo Rewrite to add weighting table to ddOutput better.
				auto ot = std::find_if(ddOut->scas.cbegin(), ddOut->scas.cend(),
					[&](const boost::shared_ptr<ddOutputSingle> &val)
				{
					if (rots.bN() > 1)
					{
						if (val->beta() < it->at(IntervalTable3dDefs::BETA_MIN) - 1.e-5) return false;
						if (val->beta() > it->at(IntervalTable3dDefs::BETA_MAX) + 1.e-5) return false;
					}
					if (rots.tN() > 1)
					{
						if (val->theta() < it->at(IntervalTable3dDefs::THETA_MIN) - 1.e-5) return false;
						if (val->theta() > it->at(IntervalTable3dDefs::THETA_MAX) + 1.e-5) return false;
					}
					if (rots.pN() > 1)
					{
						if (val->phi() < it->at(IntervalTable3dDefs::PHI_MIN) - 1.e-5) return false;
						if (val->phi() > it->at(IntervalTable3dDefs::PHI_MAX) + 1.e-5) return false;
					}
					return true;
				});
				if (ot == ddOut->scas.cend())
				{
					cerr << "Could not match rotation ("
						<< it->at(IntervalTable3dDefs::BETA_PIVOT) << ", "
						<< it->at(IntervalTable3dDefs::THETA_PIVOT) << ", "
						<< it->at(IntervalTable3dDefs::PHI_PIVOT) << ").\n";
					continue;
				}

				/* out << "Filename\tDescription\tShape Hash\tDDSCAT Version Tag\tFrequency (GHz)\t"
				"M_real\tM_imag\tAeff (um)\t"
				"Qsca_iso\tQbk_iso\tQabs_iso\tQext_iso"
				"Beta\tTheta\tPhi\tWeight\tQsca_ori\tQbk_ori\tQabs_ori\tQext_ori"<< endl;
				*/
				std::complex<double> m;
				if (ddOut->ms.size()) m = ddOut->ms.at(0);
				out << p.string() << "\t" << ddOut->description << "\t"
					<< ddOut->shapeHash.lower << "\t" << ddOut->ddvertag << "\t"
					<< ddOut->freq << "\t" << m.real() << "\t" << m.imag() << "\t"
					<< ddOut->aeff << "\t"
					<< Qsca_iso << "\t"
					<< Qbk_iso << "\t"
					<< Qabs_iso << "\t"
					<< Qext_iso << "\t"
					<< (*ot)->beta() << "\t" << (*ot)->theta() << "\t" << (*ot)->phi() << "\t"
					<< it->at(IntervalTable3dDefs::WEIGHT) << "\t"
					<< (*ot)->getStatEntry(stat_entries::QSCAM) << "\t"
					<< (*ot)->getStatEntry(stat_entries::QBKM) << "\t"
					<< (*ot)->getStatEntry(stat_entries::QABSM) << "\t"
					<< (*ot)->getStatEntry(stat_entries::QEXTM) << "\n"
					;
			}
		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


