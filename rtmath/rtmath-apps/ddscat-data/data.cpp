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
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-data\n\n";
		rtmath::debug::appEntry(argc, argv);
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string>(),"specify input file")
			("output,o", po::value<string>(), "specify output file")
			//("Mueller-method", po::value<string>()->default_value("bh"),
			//	"Specify method used to determine Mueller matrix (tmatrix, bh, ddscat). bh and ddscat are equivalent.")
			("stats", "display file stats")
			("S", "display complex amplitude matrix")
			("F", "display F")
			("Mueller", "display mueller matrix");

		po::positional_options_description p;
		p.add("input",1);
		p.add("output",2);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		string sInput, sOutput, sMuellerMethod;
		if (vm.count("input")) sInput = vm["input"].as<string>();
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		//sMuellerMethod = vm["Mueller-method"].as<string>();

		bool dispScat = false;
		bool dispS = false;
		bool dispStat = false;
		bool dispF = false;
		if (vm.count("Mueller")) dispScat = true;
		if (vm.count("F")) dispF = true;
		if (vm.count("S")) dispS = true;
		if (vm.count("stats")) dispStat = true;

		if (vm.count("help") || argc == 1 || sInput.size() == 0) {
			cerr << desc << "\n";
			return 2;
		}

		using namespace rtmath::ddscat;
		//std::function<void(const std::complex<double> Sn[4], double Snn[4][4])>
		//	mMeth;
		//rtmath::phaseFuncs::selectMueller(sMuellerMethod, mMeth);
		ddOutputSingle ddfile(sInput);

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

//		if (sOutput.size())
//			ddfile.writeFile(sOutput);
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


