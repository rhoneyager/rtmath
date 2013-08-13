/* This program is designed to read and write any type of ddscat file. It is useful for converting 
* formats and regenerating bad older data. It also is used for extracting ddscat run results 
* for comparison with other methods, like tmatrix
*/
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-data\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >(),"specify input file")
			("output,o", po::value<string>(), "specify output file")
			//("Mueller-method", po::value<string>()->default_value("bh"),
			//	"Specify method used to determine Mueller matrix (tmatrix, bh, ddscat). bh and ddscat are equivalent.")
			("stats", "display file stats")
			("S", "display complex amplitude matrix")
			("F", "display F")
			("Mueller", "display mueller matrix")
			("tag,t", "Tags the files with the standard ddscat version and run time information")
			;

		po::positional_options_description p;
		p.add("input",-1);
		//p.add("output",2);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		string sMuellerMethod;
		vector<string> vInput;
		string sOutput;
		if (vm.count("input")) vInput = vm["input"].as<vector<string> >();
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		//sMuellerMethod = vm["Mueller-method"].as<string>();

		if (!vInput.size()) doHelp("Need to specify input files");

		if (vInput.size() > 1 && vm.count("output")) doHelp("You can only specify "
			"an output file if there is a single input file.");

		bool dispScat = false;
		bool dispS = false;
		bool dispStat = false;
		bool dispF = false;
		bool tag = false;
		if (vm.count("Mueller")) dispScat = true;
		if (vm.count("F")) dispF = true;
		if (vm.count("S")) dispS = true;
		if (vm.count("stats")) dispStat = true;
		if (vm.count("tag")) tag = true;

		for (const std::string &t : vInput)
		{
			using namespace boost::filesystem;
			path p(t);
			cerr << "Processing: " << t << endl;
			if (is_directory(p) && tag) rtmath::ddscat::ddUtil::tagTARGETs(p);
			if (tag) continue;
			vector<path> pCand;
			if (is_directory(p))
				copy(recursive_directory_iterator(p,symlink_option::no_recurse), 
					recursive_directory_iterator(), back_inserter(pCand));
			else pCand.push_back(p);
			if (pCand.size() > 1 && sOutput.size()) doHelp("Only one input file allowed if "
				"output file is written.");
			for (const auto &f : pCand)
			{
				if (is_directory(f)) continue;
				string uname, meth, ext;
				Ryan_Serialization::uncompressed_name(f.string(), uname, meth);
				ext = path(uname).extension().string();
				if (ext == ".avg" || ext == ".sca" || ext == ".fml")
				{
					using namespace rtmath::ddscat;
					//std::function<void(const std::complex<double> Sn[4], double Snn[4][4])>
					//	mMeth;
					//rtmath::phaseFuncs::selectMueller(sMuellerMethod, mMeth);
					ddOutputSingle ddfile(f.string());

					if (dispStat)
					{
						cout << "Stat table:\n";
						ddfile.writeStatTable(cout);
						cout << endl;
					}

					if (dispF)
					{
						cout << "F matrix:\n";
						ddfile.writeF(cout);
						cout << endl;
					}

					if (dispS)
					{
						cout << "S matrix:\n";
						ddfile.writeS(cout);
						cout << endl;
					}

					if (dispScat)
					{
						cout << "Mueller matrix:\n";
						ddfile.writeMueller(cout);
						cout << endl;
					}

					if (sOutput.size())
						ddfile.writeFile(sOutput);

				}
			}
		}



	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


