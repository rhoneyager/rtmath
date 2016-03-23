/// Calculates kappa, beta and gamma for HW 2014 and HHT 2016 analysis.
#include <functional>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <set>
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/io.h>
#include <Ryan_Debug/registry.h>
#include <Ryan_Debug/splitSet.h>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/registry.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapefile_supplemental.h"
#include "../../rtmath/rtmath/ddscat/points.h"
#include "../../rtmath/rtmath/zeros.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "llsq.hpp"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-hogan-measure\n\n";
		const double pi = boost::math::constants::pi<double>();
		const float pif = boost::math::constants::pi<float>();
		namespace po = boost::program_options;

		po::positional_options_description p;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >()->multitoken(), "input shape file(s)")
			("output,o", po::value< string >(), "output stats file")
			("plane,p", po::value<string>()->default_value("0,1,2"), "Slice along plane(s). "
			 "Value is axis normal (0 - x, 1 - y, 2 - z).")
			("numbins,n", po::value<int>()->default_value(50), "Set number of bins")
			;

		rtmath::debug::add_options(cmdline, config, hidden);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size()) cerr << message << endl;
			exit(1);
		};
		
		rtmath::debug::process_static_options(vm);

		using namespace rtmath::ddscat::shapefile;
		using rtmath::ddscat::shapefile::shapefile;

		if (vm.count("help") || argc == 1) doHelp("");

		if (!vm.count("input")) doHelp("Need to specify input file(s).");
		vector<string> input = vm["input"].as<vector<string> >();

		string sOutput;
		if (vm.count("output")) {
			sOutput = vm["output"].as<string>();
			cerr << "Writing stats file as " << sOutput << endl;
		} else doHelp("Must specify an output file");
		string splane = vm["plane"].as<string>();
		set<int> planes;
		Ryan_Debug::splitSet::splitSet<int>(splane,planes);
		const int nBins = vm["numbins"].as<int>();
		// Column 0 is the bin start (-1/2 to 1/2)
		// Column 1 is the bin mid
		// Column 2 is the actual number of counts
		// Column 3 is the normalized number of counts. Normalized such that
		// the area under the curve is one.
		// Column 4 is the fitted kappa number of counts (normalized)
		// Column 5 is the fitted kappa number of counts (unnormalized)
		// Column 6 is the raw counts - fitted.
		Eigen::Array<float, Eigen::Dynamic, 7> hist;
		hist.resize(nBins, 7);
		hist.setZero();
		auto binStarts = hist.block(0,0,nBins,1);
		auto binMids = hist.block(0,1,nBins,1);
		auto histCounts = hist.block(0,2,nBins,1);
		auto histNorm = hist.block(0,3,nBins,1);
		auto fitNorm = hist.block(0,4,nBins,1);
		auto fitCounts = hist.block(0,5,nBins,1);
		auto diffKappa = hist.block(0,6,nBins,1);
		const float width = 1.f / (float) nBins;
		for (int i=0; i < nBins; ++i)
			hist(i,0) = (-1.f/2.f) + (((float) i) / (float) nBins);
		binMids = binStarts + (width / 2.f);
		for (const auto &ifile : input)
		{
			cerr << "Reading input shape file " << ifile << endl;
			boost::shared_ptr<shapefile> shp = shapefile::generate(ifile);
			//cerr << shp->mins << "\n\n" << shp->maxs << endl;
			for (const auto &plane : planes)
			{
				cerr << "\tPlane " << plane << endl;
				auto sx = shp->sliceAll(plane, nBins);
				//cout << (*sx) << endl;
				for (int i=0; i < sx->rows(); ++i) { // i is the bin number
					// Conveniently, the number of bins in sliceAll and in the
					// output are clamped together.
					hist(i,2) = hist(i,2) + (*sx)(i,2);
				}
			}
		}
		// Renormalize so that the integral is one.
		const float factor = histCounts.sum();
		//cout << factor << "\t" << width << endl;
		histNorm = histCounts / width / factor;

		// Perform fitting of the kappa parameter
		// A_fit(s) = (1 + kappa/3) * cos(pi s) + kappa * cos(3 pi s)
		//			= cos(pi s)		+ kappa * (cos(pi s)/3 + cos(3 pi s))
		// A_fit/cos(pi s) = 1 + kappa (-8/3 + 4 cos^2(pi s))
		// Y		= beta			+ kappaFitted * X
		// Note that the fits need an offset in s (col 0 is bin left, so add width / 2)
		// The residuals (A_fit - A(s)) are in the final column
		double alpha = 0.5, beta = 0;
		typedef Eigen::Array<float, Eigen::Dynamic, 1> AF;
		AF xs, ys, cps, cps2;
		cps = (binMids * pi).cos();
		cps2 = cps * cps;
		ys = ((histNorm/cps)-1.) / 4.;
		xs = cps2 - (2./3.);

		Eigen::Array<double, Eigen::Dynamic, 1> x, y;
		x = xs.cast<double>();
		y = ys.cast<double>();
		llsq(hist.rows(), x.data(), y.data(),
			alpha, beta);
		cerr << "alpha " << alpha << ", beta " << beta << endl;

		double kappaFitted = 0.25;
		cerr << "Fitted kappa is " << kappaFitted << ", with intercept of " << beta << endl;
		fitNorm = (cps * (1. + kappaFitted/3.))
			+ (((cps * cps * cps * 4.) - (cps * 3.))*kappaFitted);
		//fitNorm = (cps.cast<float>() * (1 + (kappaFitted / 3)));
		fitCounts = fitNorm * width * factor;
		diffKappa = histCounts - fitCounts;
		// Write the output
		ofstream out(sOutput.c_str());
		out << "Bin left\tBin Mid\tCounts\tNormalized Counts\t"
			"Fitted kappa counts (norm)\t"
			"Fitted kappa counts (unnorm)\tRaw - fitted\n";
		for (int i=0; i < hist.rows(); ++i) {
			for (int j=0; j < hist.cols(); ++j) {
				if (j) out << "\t";
				out << hist(i,j);
			}
			out << endl;
		}
		//out << hist << endl;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

