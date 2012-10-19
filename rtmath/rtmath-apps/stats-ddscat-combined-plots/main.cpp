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

//#include "../../rtmath/rtmath/ROOTlink.h"
//#include "../../rtmath/rtmath/VTKlink.h"

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
/*
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_lib_io.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
*/
#pragma warning( pop ) 
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/error/debug.h"
//#include "../../rtmath/rtmath/MagickLINK.h"
//#include "../../rtmath/rtmath/Garrett/pclstuff.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/units.h"

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
			// dipole spacing is automatically detected

			//("root", "Indicates that ROOT output is desired")
			//("csv", "Indicates that csv / tsv output is desired")
			//("png", "Produce basic png plots")
			// Add plotting var specification, axes labels, titles, ...

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
		vector<string> inputs;

		if (vm.count("input"))
		{
			cerr << "Input files / paths are:" << endl;
			for (auto it = rawinputs.begin(); it != rawinputs.end(); ++it)
				cerr << "\t" << *it << "\n";
		} else {
			cerr << "Need to specify input files or directories\n" << desc << endl;
			return 1;
		}

		// Expand directories and validate input files
		// If a directory is specified, recurse through the 
		// structure and pick up all valid shape files.
		for (auto it = rawinputs.begin(); it != rawinputs.end(); ++it)
		{
			path pi(*it);
			if (is_symlink(pi)) pi = read_symlink(pi);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi) )
			{
				vector<path> cands;
				copy(recursive_directory_iterator(pi,symlink_option::recurse), 
					recursive_directory_iterator(), back_inserter(cands));
				for (auto f = cands.begin(); f != cands.end(); ++f)
				{
					path pf = *f;
					if (is_symlink(pf)) pf = read_symlink(pf);
					if (is_directory(pf)) continue;
					path fname = pf.filename();
					if (fname.string() == "shape.dat" || 
						fname.string() == "target.out" || 
						fname.extension().string() == ".shp")
						inputs.push_back(pf.string());
				}
			} else 
			{
				inputs.push_back(*it);
			}
		}

		// gout is the global output csv file that contains all of the analyzed 
		// avg file data combined with the shape stats
		ofstream gout(vm["output"].as< string >().c_str());
		writeCSVheader(gout);

		vector<rtmath::ddscat::shapeFileStats> Stats;
		Stats.reserve(inputs.size());

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file or shape stats file
			cerr << "Processing " << *it << endl;
			boost::shared_ptr<rtmath::ddscat::shapeFileStats> sstats
				( new rtmath::ddscat::shapeFileStats );
			// Determine file type by extension
			if (it->find(".xml") == std::string::npos)
			{
				cerr << "\tShape file detected\n";
				rtmath::ddscat::shapefile shp(*it);
				rtmath::ddscat::shapeFileStats *sp = 
					new rtmath::ddscat::shapeFileStats(shp);
				sstats.reset( sp );
				cerr << "\tCalculating baseline statistics" << endl;
				sstats->calcStatsBase();
			} else {
				cerr << "\tStats file detected\n";
				rtmath::serialization::read<rtmath::ddscat::shapeFileStats>
					(*sstats, *it);
			}


			// Use boost::filesystem to iterate over all .avg files in the same directory
			path pdir = path(*it).remove_leaf();
			vector<path> vec;
			copy(directory_iterator(pdir), directory_iterator(), back_inserter(vec));

			for (auto ot = vec.begin(); ot != vec.end(); ++ot)
			{
				// Check extension
				string ext = ot->extension().string();
				if (ext != ".avg") continue;

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
				gout << *it << "," << *ot << "," << freq <<"," << aeff << "," << dSpacing << "," 
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

		//cerr << "Done." << endl;
		//shp.print(out);
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

