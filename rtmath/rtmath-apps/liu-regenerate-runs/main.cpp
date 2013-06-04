/* This program will read in Liu data and will regenerate the 
 * corresponding shape files used for running.
 */

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <list>
#include <vector>

#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/tmData.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/tmData_serialization.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/error/error.h"

void getId(const boost::filesystem::path &p,
		   std::string &ident,
		   std::string &genCmd,
		   double &dipoleSpacing)
{
	using namespace boost::filesystem;
	using namespace std;
	using namespace rtmath;

	// Open file and get TARGET line
	string sp = p.string();
	if (!exists(p)) throw debug::xMissingFile(sp.c_str());
	rtmath::ddscat::ddOutputSingle df;
	std::ifstream in(sp.c_str());
	df.readAVG(in); // Forcing read as an AVG file

	// class ddtarget : public ::rtmath::ddscat::ddOutputSingleObj
	// map is called "target"
	boost::shared_ptr<rtmath::ddscat::ddOutputSingleObj> tobj = df.getObj("target");
	string target = tobj->value();

	list<double> tparams; // This holds the numeric parameters from the end
	int filename_id = 0; // Used for bullet rosettes - from filename
	bool isRosette = false, isSnowflake = false;
	if (target.find("rosette") != string::npos) isRosette = true;
	else if (target.find("Snowflake") != string::npos) isSnowflake = true;
	else throw debug::xBadInput(sp.c_str());

	vector<string> sres;
	boost::algorithm::split(sres, target, boost::algorithm::is_any_of(" "), 
		boost::algorithm::token_compress_on);
	// From trimming, the first token in rbegin should be numeric. Pull in numbers 
	// until the first non-numeric entry is encountered.
	for (auto it = sres.rbegin(); it != sres.rend(); ++it)
	{
		try { // Totally abusing try / catch here
			double val = boost::lexical_cast<double>(*it);
			tparams.push_front(val);
		} catch (...) { break; }
	}
	// Now, if this is a rosette, look at the end of the filename to 
	// get the number of arms.
	if (isRosette)
	{
		string s = p.string();
		auto ot = s.rbegin();
		char v[2] = { 0, 0 };
		v[0] = *ot;
		ot++;
		if (*ot != '.') throw debug::xBadInput(sp.c_str());
		int n = atoi(v);
		filename_id = n;
		//tparams.push_back(double(n));
	}

	// The generation command
	ostringstream cmd;
	if (isRosette) cmd << "makerosette";
	if (isSnowflake) cmd << "makesnowflake";
	for (auto ot = tparams.begin(); ot != tparams.end(); ++ot)
		cmd << " " << *ot;
	if (isRosette) cmd << " " << filename_id; // To prevent decimal values
	genCmd = cmd.str();

	// The output file id
	// Liu places his files in freq_tempid/avg_someid[.numbullets]
	// The id field is freq-indep!
	// I need to be able to match this later, so I'll keep the filename, 
	// appending .shp to it.
	path pfile = p.filename();
	ident = pfile.string();
	ident.append(".shp");

	// The dipole spacings also vary by shape file
	dipoleSpacing = df.dipoleSpacing();
}


int main(int argc, char **argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>()->default_value("./"), 
			"Specify output directory for shapes and the dipole file")

			("inputs,i", po::value<vector<string> >(), 
			"Specify input files (avg files only)")
			("verbose,v", "Verbose output")
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
			cerr << desc << "\n";
			return 2;
		}

		if (!vm.count("inputs"))
		{
			cerr << "Need to specify input files\n";
			cerr << desc << "\n";
			return 1;
		}
		rawinputs = vm["inputs"].as<vector<string> >();

		using namespace boost::filesystem;
		outpath = vm["output"].as<string>();
		path poutpath(outpath);

		ostringstream outdipole;
		outdipole << outpath << "dipoles.tsv";
		string soutdipole = outdipole.str();
		ofstream outD(soutdipole.c_str());
		outD << "shape\tdipole_spacing\tgenCmd\n";

		// No recursion / symlink following here. Everything is treated 
		// as a valid input.

		// Can't multithread, as the make_ commands all write to the 
		// same file.
		for (auto it = rawinputs.begin(); it != rawinputs.end(); ++it)
		{
			cout << "Parsing " << *it << endl;
			double dipoleSpacing;
			string id,genCmd;
			path p(*it);
			getId(p,id,genCmd,dipoleSpacing);

			cout << id << "\t" << dipoleSpacing << "\t" << genCmd << endl;
			outD << id << "\t" << dipoleSpacing << "\t" << genCmd << endl;
			
			system(genCmd.c_str());
			boost::filesystem::rename(path("./shape.dat"), poutpath / path(id));
		}

	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		exit(1);
	}
	return 0;
}
