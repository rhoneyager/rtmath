/* This program is designed to extract tmatrix run results and combine them 
* with ddscat calculations. T-matrix outputs use tmData serialization, which 
* records temperature, frequency, source file path and any relevant statistics
* (Qext, Qsca, Qbk). The ddscat output can be taken from .avg or .sca files.
* .fml does not work yet because several ddscat stat calculations are not yet 
* implemented. 
*
* Alignment tries to match ddscat.par file paths, if possible. If unable, 
* it falls back to matching on other quantities. If .sca files are given, it 
* will match individual orientations along with isotropic ensemble results. 
* If .avg files are provided, then only isotropic ensemble results are provided. 
* 
* The csv output provides fields as follows:
- source
- isotropic / specific orientation flags
- orientation beta
- orientation theta
- orientation phi
- frequency
- effective radius
- dipole spacing
- temperature
- nu
- volmeth
- dielmeth
- shapemeth
- anglemeth
- phi (can be a number or 'mean')
- ddscat qsca
- ddscat qext
- ddscat qbk
- tmatrix qsca
- tmatrix qext
- tmatrix qbk
- stats fields
- max distance
- aspect ratios (abs and rms)
- effective radii
- volume fractions
- volumes and surface areas
- cross-sectional area (in future)
* ddscat S matrices cannot be extracted easily without fml files, so these are ignored for 
* both ddscat and tmatrix.
* shape stats are preserved for better analysis. stats are extracted from 
* the tmatrix run files (conveniently)
*/

#include <vector>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/tmData.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/tmData_serialization.h"
#include "../../rtmath-libs/rtmath-mie/mie-serialization.h"
#include "../../rtmath-libs/rtmath-mie/mie.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/error/error.h"

void writeCSVheader(std::ostream &out)
{
	using namespace std;
	out << "source,isoflag,oribeta,oritheta,oriphi,frequency,wavelength,sizep,"
		<< "dipole_spacing,T,nu,reff_real,reff_imag,volmeth,dielmeth,shapemeth,"
		<< "aeff_dd,aeff_tm,aspect_ratio,dd_qext,tm_qext,"
		<< "dd_qsca,tm_qsca,dd_qbk,tm_qbk,dd_walb,tm_walb"
		<< std::endl;
}

bool prefixFoldersOnly = false;

class dataset
{
public:
	dataset() : id("") {}
	dataset(const std::string &prefix) : id(prefix) {}
	std::string id;
	std::vector<boost::filesystem::path> ddres;
	std::vector<boost::filesystem::path> tmres;


	static std::string getPrefix(const boost::filesystem::path &filename, size_t ignore_prefix_length = 0)
	{
		// Prefix is the parent path and the first part of the filename.
		// Parent path is included to allow recursion without naming conflicts
		using namespace std;
		using namespace boost::filesystem;
		path pleaf = filename.filename();
		path proot = filename.parent_path();
		if (prefixFoldersOnly) return proot.string();
		// holly shape files are like 1mm14shape.txt
		// liu shape files are like avg_5000.shp
		// t-matrix results are like 1mm14shape.txt-tmatrix- ... .xml (.bz2)
		// holly avg files follow 1mm14_t_f.avg
		// liu avg files follow avg_3000[.*]
		// sca files are not currently implemented
		string sleaf = pleaf.string(); // the filename
		// Remove the first ignore_prefix_length characters from the prefix calculation
        if (ignore_prefix_length)
        {
                if (sleaf.size() < ignore_prefix_length) throw;
                sleaf = sleaf.substr(ignore_prefix_length);
        }

		size_t pund = sleaf.find('_'); // May be liu or holly
		size_t pshape = sleaf.find("shape"); // Only Holly
		size_t pshp = sleaf.find(".shp"); // Only Liu
		size_t pavg = sleaf.find("avg_"); // Only Liu

		bool isLiu = false, isHolly = false;
		if (pshp != string::npos) isLiu = true;
		if (pavg != string::npos) isLiu = true;
		if ((pshape != string::npos) && !isLiu) isHolly = true;
		if ((pund != string::npos) && !isLiu) isHolly = true;

		// Based on detection criteria, perform appropriate truncation
		if (isLiu)
		{
			// Liu avg files need no manipulation.
			// Remove .shp - pshp for raw shapes and tmatrix results
			if (pshp != string::npos)
				sleaf = sleaf.substr(0,pshp);
			// Raw liu shapes, tmatrix results re now truncated
		}
		if (isHolly)
		{
			size_t pos = 0;
			if ((pund != string::npos) && (pavg != string::npos)) pund = string::npos;
			if (pund == string::npos && pshape != string::npos) pos = pshape;
			else if (pshape == string::npos && pund != string::npos) pos = pund;
			else if (pshape == pund == string::npos) throw;
			else pos = (pund < pshape) ? pund : pshape;
			sleaf = sleaf.substr(0,pos);
		}

		return sleaf;
	}
	static bool isValid(const boost::filesystem::path &filename)
	{
		using namespace boost::filesystem;
		using namespace std;
		if (is_directory(filename)) return false;
		path pleaf = filename.filename();
		path ext = pleaf.extension();
		if (ext.string() == ".avg") return true;
		if (pleaf.string().find("avg") != string::npos)
			if (pleaf.string().find("tmatrix") == string::npos) return true;
		if (pleaf.string().find("tmatrix") != string::npos
			&& pleaf.string().find(".xml") != string::npos) return true;
		//if (pleaf.string().find("shape.txt") != string::npos) return true;
		//if (ext.string() == ".shp") return true;
		//if (pleaf.string().find("shape.dat") != string::npos) return true;
		return false;
	}
	static bool isAvg(const boost::filesystem::path &filename)
	{
		using namespace boost::filesystem;
		using namespace std;
		path pleaf = filename.filename();
		path ext = pleaf.extension();
		if (ext.string() == ".avg") return true;
		if (pleaf.string().find("avg") != string::npos)
			if (pleaf.string().find("tmatrix") == string::npos) return true;
		return false;
	}
	static bool isTmm(const boost::filesystem::path &filename)
	{
		if (!isValid(filename)) return false;
		if (isAvg(filename)) return false;
		return true;
	}
};


int main(int argc, char** argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>()->default_value("mergeresults.csv"), 
			"Specify output csv file for merge results")

			("inputs,i", po::value<vector<string> >(), 
			"Specify input files (tmatrix serialization, .avg)")
			("match-folders-only,f", "Match only based on parent folder name")
			("ignore-prefix-length", po::value<size_t>()->default_value(0),
             "Ignore the first n characters in the filename")
			("verbose,v", "Verbose output")
			("tolerance,t", po::value<double>()->default_value(0.0001),
			"Specify tolerance for frequency matching")
			;

		po::positional_options_description p;
		p.add("inputs",-1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		vector<string> rawinputs; // May be expanded by os.

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 2;
		}

		bool verbose = false;
		if (vm.count("verbose")) verbose = true;
		size_t ignore_prefix_length = vm["ignore-prefix-length"].as<size_t>();

		if (!vm.count("inputs"))
		{
			cerr << "Need to specify input files\n";
			cerr << desc << "\n";
			return 1;
		}
		rawinputs = vm["inputs"].as<vector<string> >();

		if (vm.count("match-folders-only"))
			prefixFoldersOnly = true;

		double tol = vm["tolerance"].as<double>();

		using namespace boost::filesystem;

		// sca and fml files have no reference to the originating ddscat runs. However, 
		/* they can be matched by:
		1 - filename
		2 - using the TARGET field - it's the shape.dat title
		3 - header properties (# of dipoles, freq, d, aeff)
		*/

		map<string,dataset> data;

		auto insertMapping = [&](const boost::filesystem::path &p)
		{
			if (dataset::isValid(p) == false) return;
			std::string prefix = dataset::getPrefix(p, ignore_prefix_length);
			if (!data.count(prefix))
				data[prefix] = std::move(dataset(prefix));
			if (dataset::isAvg(p))
				data[prefix].ddres.push_back(p);
			if (dataset::isTmm(p))
				data[prefix].tmres.push_back(p);
		};

		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				if (is_directory(pf))
					return pf;
				else
					return p;
			} else {
				return p;
			}
		};

		// Expand directories and validate input files
		// If a directory is specified, recurse through the 
		// structure and pick up all valid shape files.
		for (auto it = rawinputs.begin(); it != rawinputs.end(); ++it)
		{
			path pi(*it);
			pi = expandSymlinks(pi);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi) )
			{
				vector<path> cands;
				copy(recursive_directory_iterator(pi,symlink_option::recurse), 
					recursive_directory_iterator(), back_inserter(cands));
				for (auto f = cands.begin(); f != cands.end(); ++f)
				{
					path pf = *f;
					pf = expandSymlinks(pf);
					insertMapping(pf); // already does dir checking
				}
			} else 
			{
				insertMapping(*it);
			}
		}

		// Now that the mappings are established, we can load the 
		// avg and tmm files and perform the computations and 
		// merging operations.

		ofstream gout(vm["output"].as<string>().c_str());
		// Write csv header
		writeCSVheader(gout);

		for (auto it = data.begin(); it != data.end(); ++it)
		{
			cerr << "Processing " << it->first << endl;
			if (it->second.ddres.size() == 0) continue;
			if (it->second.tmres.size() == 0) continue;

			// A cache is created for each object to prevent multiple loads.
			// It is dumped at the end of reading each data set.
			map<std::string, std::vector<rtmath::tmatrix::tmData> > tmcache;
			map<std::string, rtmath::ddscat::ddOutputSingle> ddcache;

			for (auto st = it->second.ddres.begin(); 
				st != it->second.ddres.end(); ++st)
			{
				using namespace rtmath::ddscat;
				cerr << "\t" << *st << std::endl;
				// Load the dd avg file
				// Read files only once. Cache the results.
				ddOutputSingle ddfile;
				if (ddcache.count(st->string()))
				{
					ddfile = ddcache.at(st->string());
				} else {
					ddfile.readFile(st->string(), ".avg");
					ddcache[st->string()] = ddfile;
				}
				double dSpacing = ddfile.dipoleSpacing();
				double aeff = ddfile.aeff();
				double freq = rtmath::units::conv_spec("um","GHz").convert(ddfile.wave());

				double qbk = ddfile.getStatEntry(QBKM);
				double qsca = ddfile.getStatEntry(QSCAM);
				double qabs = ddfile.getStatEntry(QABSM);
				double qext = ddfile.getStatEntry(QEXTM);
				double g1 = ddfile.getStatEntry(G1M);
				double g2 = ddfile.getStatEntry(G2M);

				// Iterate over all tmatrix and mie runs
				for (auto ot = it->second.tmres.begin(); 
					ot != it->second.tmres.end(); ++ot)
				{
					if (verbose)
						cerr << "\t\tcand\t" << *ot << std::endl;
					// Read files only once. Cache the results.
					std::vector<rtmath::tmatrix::tmData> td;
					//rtmath::tmatrix::tmData td;
					if (tmcache.count(ot->string()))
					{
						td = tmcache.at(ot->string());
					} else {
						rtmath::serialization::read<std::vector<rtmath::tmatrix::tmData> >
							(td, ot->string());
						tmcache[ot->string()] = td;
					}

					if (verbose)
						cerr << "There are " << td.size() << " candidates." << endl;
					// Iterate through the tmatrix combinations
					for (auto pt = td.begin(); pt != td.end(); ++pt)
					{
						if (verbose)
							cerr << pt->freq << "\t" << freq << std::endl;
						if (abs(pt->freq - freq) > tol ) continue; // 0.0001

						cerr << "\t\t" << *ot << std::endl;

						// For each tmatrix / mie file, the results are already 
						// combined according to isotropic orientation!
						double tQscam = pt->tstats->stats.at("Qsca");
						double tQextm = pt->tstats->stats.at("Qext");
						double tQbkm = pt->tstats->stats.at("Qbk");


						// Write out the resultant values to a csv file.
						/*
						out << "source,isoflag,oribeta,oritheta,oriphi,frequency,wavelength,sizep,"
						<< "dipole_spacing,T,nu,reff_real,reff_imag,volmeth,dielmeth,shapemeth,"
						<< "aeff_dd,aeff_tm,aspect_ratio,dd_qext,tm_qext,"
						<< "dd_qsca,tm_qsca,dd_qbk,tm_qbk,dd_walb,tm_walb"
						<< std::endl;
						*/
						if (pt->data.size())
						{
							auto qt = pt->data.begin();
							gout << it->first << ",iso,iso,iso,iso,";
							gout << freq << "," << ddfile.wave() << "," << pt->sizep << ","
								<< dSpacing << "," << pt->T << "," << pt->nu << "," 
								<< pt->reff.real() << "," << pt->reff.imag() << ","
								<< pt->volMeth << "," << pt->dielMeth << "," << pt->shapeMeth << ","
								<< aeff << "," << qt->get()->tm->base->axi << ","
								<< qt->get()->tm->base->eps << ","
								<< qext << "," << tQextm << ","
								<< qsca << "," << tQscam << "," 
								<< qbk << "," << tQbkm << ","
								<< qsca/qext << "," << tQscam/tQextm;
						
							gout << std::endl;
						}
						if (pt->miedata.size())
						{
							auto qt = pt->miedata.begin();
							gout << it->first << ",iso,iso,iso,iso,";
							gout << freq << "," << ddfile.wave() << "," << pt->sizep << ","
								<< dSpacing << "," << pt->T << "," << pt->nu << "," 
								<< pt->reff.real() << "," << pt->reff.imag() << ","
								<< pt->volMeth << "," << pt->dielMeth << "," << pt->shapeMeth << ","
								<< aeff << "," << qt->get()->mc->base->axi << ","
								<< "1.0000" << ","
								<< qext << "," << tQextm << ","
								<< qsca << "," << tQscam << "," 
								<< qbk << "," << tQbkm << ","
								<< qsca/qext << "," << tQscam/tQextm;
						
							gout << std::endl;
						}
					}
				}
			}
		}

	}
	catch (std::exception &e)
	{
		cerr << "Exception caught\n";
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}
