/* This program will read in a ddscat.par file and will 
 * regenerate the appropriate shape.dat file using a 
 * provided ddscat calltarget program
 */

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <list>
#include <vector>
#include <cstdlib>
#include <Ryan-Debug/debug.h>
#include <Ryan-Serialization/serialization.h>

#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/tmData.h"
#include "../../rtmath/rtmath/Serialization/tmData_serialization.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/error/error.h"

void convert(const boost::filesystem::path &p,
		   std::string &outshape)
{
	using namespace boost::filesystem;
	using namespace std;
	using namespace rtmath;

	// Open file and get shape params
	string ps = p.string();
	rtmath::ddscat::ddPar par(ps);
	string shape;
	double shparams[3];
	par.getShape(shape);
	// If already FROM_FILE, throw here.
	if (shape == "FROM_FILE") throw debug::xBadInput(ps.c_str());

	for (size_t i=0;i<3;i++)
		shparams[i] = par.shpar(i);

	// Command generation involves creating a file to pipe to calltarget,
	// invoking calltarget and then moving the target.out file to the desired 
	// location.

	path tfile = boost::filesystem::unique_path();
	string stfile = tfile.string();
	ofstream out(stfile.c_str()); // Holds the construction script
	out << shape << endl;
	out << shparams[0] << " " << shparams[1] << " " << shparams[2] << endl;
	out << "0 0 0" << endl;

	if (exists(path("target.out"))) throw rtmath::debug::xPathExistsWrongType("shape.dat");

	ostringstream cmd;
	cmd << "calltarget < " << stfile;
	string scmd = cmd.str();
	system(scmd.c_str());
	boost::filesystem::remove(tfile);
	outshape = string(ps).append(".shp");
	boost::filesystem::rename(path("target.out"), path(outshape));

	rtmath::ddscat::ddPar parnew = par;
	parnew.setShape("FROM_FILE");
	string psnew = ps;
	psnew.append(".new.par");
	parnew.writeFile(psnew);
}

int main(int argc, char** argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("inputs,i", po::value<vector<string> >(), 
			"Specify input files (par files only). Outputs will be "
			 "prefixed with the name of the input.")
			;

		po::positional_options_description p;
		p.add("inputs",-1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		vector<string> rawinputs; // May be expanded by os.
		string outpath;

		if (vm.count("help") || argc == 1) {
			cerr << desc << endl;
			return 2;
		}

		if (!vm.count("inputs"))
		{
			cerr << "Need to specify input files\n";
			cerr << desc << endl;
			return 3;
		}
		rawinputs = vm["inputs"].as<vector<string> >();

		using namespace boost::filesystem;

		// No recursion / symlink following here. Everything is treated 
		// as a valid input.

		// Can't multithread, as the make_ commands all write to the 
		// same file.
		for (auto it = rawinputs.begin(); it != rawinputs.end(); ++it)
		{
			cout << "Parsing " << *it << endl;
			path p(*it);
			string outshape;
			convert(p, outshape);
			cout << "\t" << outshape << endl;
		}
	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
