/* This program is designed to read ddscat.avg files in conjunction with relevant shape statistics. It can either 
 * read precalculated statistics files or it can calculate the statistics from a shape.dat file. It also needs 
 * the interdipole spacing and temperature.
 *
 * This program can extract backscatter and scattering cross-sectional data and make a plot versus other 
 * statistics-derived shape quantities. It will support two axes plus color for these plots.
 */

#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN

//#include <Eigen/
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <valarray>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
//#include "../../rtmath/rtmath/MagickLINK.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/error/debug.h"
class dataset
{
public:
	dataset() : id("") {}
	dataset(const std::string &prefix)
		: id(prefix)
	{
	}
	std::string id;
	boost::filesystem::path shape;
	std::vector<boost::filesystem::path> avgs;

	static std::string getPrefix(const boost::filesystem::path &filename)
	{
		// Prefix is the parent path and the first part of the filename.
		// Parent path is included to allow recursion without naming conflicts
		using namespace std;
		using namespace boost::filesystem;
		path pleaf = filename.filename();
		// shape files are like 1mm14shape.txt
		// avg files are 1mm14_t_f.avg
		// So, search for first occurance of '_' and 'shape'. Truncate.
		string sleaf = pleaf.string();
		size_t pund = sleaf.find('_');
		size_t pshape = sleaf.find("shape");
		size_t pos = 0;
		if (pund == string::npos && pshape) pos = pshape;
		else if (pshape == string::npos && pund) pos = pund;
		else if (pshape == pund == string::npos) throw;
		else pos = (pund < pshape) ? pund : pshape;
		string prefix = filename.parent_path().string();
		prefix.append("/");
		prefix.append(sleaf.substr(0,pos));
		return prefix;
	}
	static bool isValid(const boost::filesystem::path &filename)
	{
		using namespace boost::filesystem;
		using namespace std;
		if (is_directory(filename)) return false;
		path pleaf = filename.filename();
		path ext = pleaf.extension();
		if (ext.string() == ".avg") return true;
		if (pleaf.string().find("shape.txt") != string::npos) return true;
		if (ext.string() == ".shp") return true;
		if (pleaf.string().find("shape.dat") != string::npos) return true;
		return false;
	}
	static bool isAvg(const boost::filesystem::path &filename)
	{
		using namespace boost::filesystem;
		using namespace std;
		path pleaf = filename.filename();
		path ext = pleaf.extension();
		if (ext.string() == ".avg") return true;
		return false;
	}
	static bool isShape(const boost::filesystem::path &filename)
	{
		if (!isValid(filename)) return false;
		if (isAvg(filename)) return false;
		return true;
	}
};

void writeCSVheader(std::ostream &out)
{
	using namespace std;
	/*	gout << *it << "," << *ot << "," << freq <<"," << aeff << "," << dSpacing << "," 
			<< qbk << "," << qsca << "," << qabs << "," << qext << "," << g1 << ","
			<< g2 << "," << sView->aeff_dipoles_const() << "," << sView->f_circum_sphere() << ","
			<< sView->max_distance() << "," 
			<< sRot->max().get(2,0,0) << "," 
			<< sRot->rms_mean().get(2,0,0) << "," << sRot->mom1(0).get(2,0,0) << ","
			<< sRot->mom2(0).get(2,0,0) << "," << rot->skewness.get(2,0,0) << ","
			<< rot->kurtosis.get(2,0,0) << "," 
			<< sRot->max().get(2,1,0) << "," 
			<< sRot->rms_mean().get(2,1,0) << "," << sRot->mom1(0).get(2,1,0) << ","
			<< sRot->mom2(0).get(2,1,0) << "," << rot->skewness.get(2,1,0) << ","
			<< rot->kurtosis.get(2,1,0) << ","
			<< sRot->max().get(2,2,0) << "," 
			<< sRot->rms_mean().get(2,2,0) << "," << sRot->mom1(0).get(2,2,0) << ","
			<< sRot->mom2(0).get(2,2,0) << "," << rot->skewness.get(2,2,0) << ","
			<< rot->kurtosis.get(2,2,0) << ","
			<< sRot->max().get(2,3,0) << "," 
			<< sRot->rms_mean().get(2,3,0) << "," << sRot->mom1(0).get(2,3,0) << ","
			<< sRot->mom2(0).get(2,3,0) << "," << rot->skewness.get(2,3,0) << ","
			<< rot->kurtosis.get(2,3,0) << "," 
					
			<< rot->as_rms.get(2,0,0) << "," << rot->as_rms.get(2,0,1) << "," << rot->as_rms.get(2,0,2) << ","
			<< rot->as_rms.get(2,1,0) << "," << rot->as_rms.get(2,1,1) << "," << rot->as_rms.get(2,1,2) << ","
			<< rot->as_rms.get(2,2,0) << "," << rot->as_rms.get(2,2,1) << "," << rot->as_rms.get(2,2,2) 
			<< std::endl; 
		*/
	out << "shape,avg,freq,aeff,dSpacing,Qbk,Qsca,Qabs,Qext,g1,g2,aeff_dipoles_const,"
		<< "f_circum_sphere,max_distance,"
		<<"max_x,rms_mean_x,mom1_x,mom2_x,skewness_x,kurtosis_x,"
		<<"max_y,rms_mean_y,mom1_y,mom2_y,skewness_y,kurtosis_y,"
		<<"max_z,rms_mean_z,mom1_z,mom2_z,skewness_z,kurtosis_z,"
		<<"max_R,rms_mean_R,mom1_R,mom2_R,skewness_R,kurtosis_R,"
		<< "AS_rms_xx,AS_rms_xy,AS_rms_xz,AS_rms_yx,AS_rms_yy,AS_rms_yz,AS_rms_zx,AS_rms_zy,AS_rms_zz"
		<< std::endl;
}

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-stats-ddscat-combined-plots\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files")
			("output,o", po::value<string>()->default_value("results.csv"), "output csv file")
			//("root", "Indicates that ROOT output is also desired")

			("disable-qhull", "Disable qhull calculations for the shapes.");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		if (vm.count("disable-qhull"))
			rtmath::ddscat::shapeFileStats::doQhull(false);

		vector<string> rawinputs = vm["input"].as< vector<string> >();
		map<string,dataset> data;

		if (!vm.count("input"))
		{
			cerr << "Need to specify input files or directories\n" << desc << endl;
			return 1;
		}

		auto insertMapping = [&](const boost::filesystem::path &p)
		{
			if (dataset::isValid(p) == false) return;
			std::string prefix = dataset::getPrefix(p);
			if (!data.count(prefix))
				data[prefix] = std::move(dataset(prefix));
			if (dataset::isShape(p))
				data[prefix].shape = p;
			if (dataset::isAvg(p))
				data[prefix].avgs.push_back(p);
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

		// gout is the global output csv file that contains all of the analyzed 
		// avg file data combined with the shape stats
		ofstream gout(vm["output"].as< string >().c_str());
		writeCSVheader(gout);

		for (auto it = data.begin(); it != data.end(); ++it)
		{
			// Load the shape file or shape stats file
			cerr << "Processing " << it->first << endl;
			if (it->second.shape.string() == "") continue;
			if (it->second.avgs.size() == 0) continue;
			boost::shared_ptr<rtmath::ddscat::shapeFileStats> sstats
				= rtmath::ddscat::shapeFileStats::genStats(it->second.shape.string());

			for (auto ot = it->second.avgs.begin(); ot != it->second.avgs.end(); ++ot)
			{
				cerr << *ot << std::endl;

				using namespace rtmath::ddscat;
				// This file is a .avg file. Read it.
				ddOutputSingle ddfile(ot->string());
				// Extract interdipole spacing, aeff and frequency
				double dSpacing = ddfile.dipoleSpacing();
				double aeff = ddfile.aeff();
				double freq = rtmath::units::conv_spec("um","GHz").convert(ddfile.wave());

				
				double qbk = ddfile.getStatEntry(QBKM);
				double qsca = ddfile.getStatEntry(QSCAM);
				double qabs = ddfile.getStatEntry(QABSM);
				double qext = ddfile.getStatEntry(QEXTM);
				double g1 = ddfile.getStatEntry(G1M);
				double g2 = ddfile.getStatEntry(G2M);

				// Create the stats views - everything wil be in physical units
				boost::shared_ptr<shapeFileStatsDipoleView> sView
					(new shapeFileStatsDipoleView(sstats, dSpacing));
				auto rot = sstats->calcStatsRot(0,0,0);
				boost::shared_ptr<shapeFileStatsRotatedView> sRot
					(new shapeFileStatsRotatedView(rot, dSpacing));


				// Write the csv files
				gout << it->first << "," << *ot << "," << freq <<"," << aeff << "," << dSpacing << "," 
					<< qbk << "," << qsca << "," << qabs << "," << qext << "," << g1 << ","
					<< g2 << "," << sView->aeff_dipoles_const() << "," << sView->f_circum_sphere() << ","
					<< sView->max_distance() << "," 
					<< sRot->max().get(2,0,0) << "," 
					<< sRot->rms_mean().get(2,0,0) << "," << sRot->mom1(0).get(2,0,0) << ","
					<< sRot->mom2(0).get(2,0,0) << "," << rot->skewness.get(2,0,0) << ","
					<< rot->kurtosis.get(2,0,0) << "," 
					<< sRot->max().get(2,1,0) << "," 
					<< sRot->rms_mean().get(2,1,0) << "," << sRot->mom1(0).get(2,1,0) << ","
					<< sRot->mom2(0).get(2,1,0) << "," << rot->skewness.get(2,1,0) << ","
					<< rot->kurtosis.get(2,1,0) << ","
					<< sRot->max().get(2,2,0) << "," 
					<< sRot->rms_mean().get(2,2,0) << "," << sRot->mom1(0).get(2,2,0) << ","
					<< sRot->mom2(0).get(2,2,0) << "," << rot->skewness.get(2,2,0) << ","
					<< rot->kurtosis.get(2,2,0) << ","
					<< sRot->max().get(2,3,0) << "," 
					<< sRot->rms_mean().get(2,3,0) << "," << sRot->mom1(0).get(2,3,0) << ","
					<< sRot->mom2(0).get(2,3,0) << "," << rot->skewness.get(2,3,0) << ","
					<< rot->kurtosis.get(2,3,0) << "," 
					
					<< rot->as_rms.get(2,0,0) << "," << rot->as_rms.get(2,0,1) << "," << rot->as_rms.get(2,0,2) << ","
					<< rot->as_rms.get(2,1,0) << "," << rot->as_rms.get(2,1,1) << "," << rot->as_rms.get(2,1,2) << ","
					<< rot->as_rms.get(2,2,0) << "," << rot->as_rms.get(2,2,1) << "," << rot->as_rms.get(2,2,2) 
					<< std::endl; 
			}

		}

	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

