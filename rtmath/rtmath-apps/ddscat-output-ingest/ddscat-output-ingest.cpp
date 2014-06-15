// This program generates an ingest script (that in turn calls rtmath-ddscat-output)
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/macros.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-output-ingest\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >()->multitoken(),"specify input directory or file.")
			("output,o", po::value<string>()->default_value("ingest.tcsh"), "specify output script")
			("force-flake-type,t", po::value<string>(), "forces flake type (dendrite, ar_correct_main, ...")
			("force-ddscat-version,v", po::value<string>(), "force ddscat version")
			("force-polarization,p", po::value<string>(), "force polarization (lin,rhc,...)")
			;
		po::positional_options_description p;
		p.add("input",-1);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

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
		else doHelp("Need to specify input");
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		else doHelp("Need to specify output");

		ofstream ofile(sOutput.c_str());

		ofile << "#!/bin/tcsh\n# Automatically-generated ingest script\n";
		ofile << endl << endl;

		string runclass;
		bool default_runclass = false;
		if (vm.count("force-flake-type")) {
			runclass = vm["force-flake-type"].as<string>();
			default_runclass = true;
		}
		bool default_pol = false, default_ver = false;
		string pol, ddver;
		if (vm.count("force-polarization")) {
			pol = vm["force-polarization"].as<string>();
			default_pol = true;
		}
		if (vm.count("force-ddscat-version")) {
			ddver = vm["force-ddscat-version"].as<string>();
			default_ver = true;
		}

		using namespace boost::filesystem;
		size_t num = 0;
		size_t mxnum = vsInput.size();
		for (const auto &i : vsInput)
		{
			num++;
			path pInput(i);
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
			path ps = expandSymlinks(pInput);
			cout << "Processing: " << pInput << " --- # " << num << " / " << mxnum << endl;
			//if (is_regular_file(ps) && !Ryan_Serialization::known_format(ps)) continue;

			auto isYes = [](bool def) -> bool
			{
				std::string res;
				getline(cin, res);
				if (!res.size()) return def;
				if (res == "\n") return def;
				if (res[0] == '1' || res[0] == 'y' || res[0] == 'Y' 
						|| res[0] == 'T' || res[0] == 't') return true;
				return false;
			};
			using namespace std;
			string hostname;
			vector<string> tags;
			if (!default_runclass)
			{
				cout << " Type of flake (dendrite,prolate,...): ";
				cin >> runclass;
				getline(cin, ddver); // because of the cin read
			}
			string cltag("flake_classification=");
			cltag.append(runclass);
			tags.push_back(cltag);
			if (!default_ver) {
				cout << " DDSCAT version [7.3.0_130527_r1-intel-openmp-checked]: ";
				getline(cin, ddver);
			}
			if (!ddver.size()) ddver = "7.3.0_130527_r1-intel-openmp-checked";
			cout << " Hostname (rosette,...) [unknown]: ";
			getline(cin,hostname);
			if (!hostname.size()) hostname = "unknown";
			if (!default_pol) {
				cout << " Polarization (lin,rhc,...) [lin]: ";
				getline(cin,pol);
				if (!pol.size()) pol = "lin";
			}
			string poltag("pol=");
			poltag.append(pol);
			tags.push_back(poltag);
			cout << " Is this run decimated [no]? ";
			if (isYes(false)) {
				string s;
				int level = 2, threshold = 0;
				cout << "  Decimation level [2]: ";
				getline(cin, s);
				if (!s.size() || s == "\n") level = 2;
				else level = M_ATOI(s.c_str());
				cout << "  Does the decimation work on a threshold [yes]? ";
				bool isThres = true;
				if (isThres = isYes(true)) {
					threshold = (level*level*level)/2;
					cout << "  Decimation threshold [" << threshold << "]: ";
					getline(cin,s);
					if (!s.size() || s == "\n") {} else threshold = M_ATOI(s.c_str());
				}
				ostringstream out;
				out << "decimation=" << level;
				if (isThres)
					out << "_" << threshold;
				else out << "_scaled";
				string stag(out.str());
				tags.push_back(stag);
			} else {
				tags.push_back("decimation=none");
			}
			cout << " Is this run perturbed [no]? ";
			if (isYes(false)) {
				cout << "  Perturbation type [internal]: ";
				string ptype;
				getline(cin,ptype);
				if (!ptype.size() || ptype == "\n") ptype = "internal";
				cout << "  Perturbation threshold and number (i.e. 3_1): ";
				string pthres;
				getline(cin,pthres);
				ostringstream out;
				out << "perturbation=" << ptype << "_" << pthres;
				tags.push_back(out.str());
			} else tags.push_back("perturbation=none");
			cout << " Suffix (_thorough?): ";
			string suffix;
			getline(cin,suffix);

			cout << endl;

			ofile << "echo " << ps.string() << " - " << num << " / " << mxnum << endl;
			ofile << "set outfull=" << runclass << "_`basename -s .tar.bz2 " 
				<< ps.string() << "`" << suffix << "-ori-fml.hdf5\n";
			ofile << "set outori=" << runclass << "_`basename -s .tar.bz2 " 
				<< ps.string() << "`" << suffix << "-ori.hdf5\n";
			ofile << "mkdir extract\n";
			path pextract;
			if (is_directory(ps))
			{
				// Already extracted
				pextract = ps;
			} else { //if (Ryan_Serialization::known_format(ps)) {
				ofile << "tar xjf " << ps.string() << " -C extract/ --strip-components=1\n";
				ofile << "if ( $? ) then\n"
					"echo " << ps.string() << " >> bad_ingest.log\n"
					"endif" << endl;
				pextract = "extract/*";
			}

			ofile << "rtmath-ddscat-output -i " << pextract.string() << " --write-shapes=0 "
				"-H " << hostname << " --force-ddscat-detected-version " 
				<< ddver << " -d " << runclass << " ";
			for (const auto &t : tags)
				ofile << "-t \"" << t << "\" ";
			ofile << "-o hdf-full/$outfull --output-aux hdf-ori/$outori" << endl;

			ofile << "if ( $? ) then\n"
				"rm hdf-full/$outfull hdf-ori/$outori\n"
				"echo " << ps.string() << " >> bad_ingest.log\n"
				"endif" << endl;

			ofile << "rm -rf extract" << endl << endl;
		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


