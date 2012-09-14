#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN

#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/VTKlink.h"
#include "../../rtmath/rtmath/MagickLINK.h"

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <valarray>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/tmData.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/tmData_serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/coords.h"


int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-plot-backscatter\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files")

			//("isotropic", "Do isotropic ensemble weighting")

			("output-prefix,o", po::value<string>()->default_value("output"), "Specify output filename prefix for plots");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		vector<string> inputs = vm["input"].as< vector<string> >();
		if (vm.count("input"))
		{
			cerr << "Input files are:" << endl;
			for (auto it = inputs.begin(); it != inputs.end(); it++)
				cerr << "\t" << *it << "\n";
		} else {
			cerr << "Need to specify input files.\n" << desc << endl;
			return 1;
		}

		string ofname = vm["output-prefix"].as<string>();
		ofname.append("-iso-ensemble-backscatter.png");

		// Validate input files
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
				throw rtmath::debug::xPathExistsWrongType(it->c_str());
		}

		vector< boost::shared_ptr<rtmath::tmatrix::tmData> > tmd;
		tmd.reserve(inputs.size());

		vector< boost::shared_ptr<map<double,matrixop> > > isos;
		vector< double > backscatters;
		isos.reserve(inputs.size());
		backscatters.reserve(inputs.size());

		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			// Load the shape file or shape stats file
			cerr << "Processing " << *it << endl;
			boost::shared_ptr<tmatrix::tmData> tm
				( new tmatrix::tmData );
			rtmath::serialization::read<tmatrix::tmData >
				(*tm, *it);
			tmd.push_back(tm);

			// Select only for copol mode (phi-phi0 = 0)
			// Define dtheta = abs(thet-thet0)
			// Arrange into map of (alpha,beta)=coords and submap (dtheta,P[4][4])
			map<rtmath::coords::cyclic<double>, boost::shared_ptr<map<double,matrixop> > > copol;
			for (auto ot = tm->data.begin(); ot != tm->data.end(); ++ot)
			{
				if ( abs( (*ot)->invars.phi - (*ot)->invars.phi0 ) > 0.01 ) continue;

				rtmath::coords::cyclic<double> c(2,(*ot)->invars.alpha,(*ot)->invars.beta);
				//map<double,matrixop> sm;
				double dtheta = abs( (*ot)->invars.thet - (*ot)->invars.thet0 );
				matrixop P(2,4,4);
				P.fromDoubleArray(&((*ot)->res.P[0][0]));

				boost::shared_ptr<map<double,matrixop> > cm;
				if (copol.count(c))
				{
					cm = copol[c];
				} else {
					cm = boost::shared_ptr<map<double,matrixop> >(new map<double,matrixop>);
					copol[c] = cm;
				}
				
				if (!cm->count(dtheta))
					cm->insert(pair<double,matrixop>(dtheta,P));
			}

			// Now do orientational averaging
			//size_t n = copol.size();
			size_t n=1;
			boost::shared_ptr<map<double,matrixop> > iso(new map<double,matrixop>);

			//for (auto ot = copol.begin(); ot != copol.end(); ++ot)
			for (auto ot = copol.begin(); ot == copol.begin(); ++ot)
			{
				for (auto pt = ot->second->begin(); pt != ot->second->end(); ++pt)
				{
					double dtheta = pt->first;
					matrixop &P = pt->second;
					matrixop rt = P * (1.0 / (double) n);
					if (!iso->count(dtheta))
					{
						iso->insert(pair<double,matrixop>(dtheta,rt));
					} else {
						matrixop &a = iso->at(dtheta);
						a = a + rt;
					}
				}
			}

			double backscatter = iso->rbegin()->second.get(2,0,0);

			// Store iso and backscatter for plotting
			isos.push_back(iso);
			backscatters.push_back(backscatter);
		}

		// Creating plot of several orientationally-averaged phase functions
		// Ensemble averaging has already been performed.

		// These exist to hold the shared_arrays and release then after the ROOT routines are called
		set<boost::shared_array<double> > xs;
		set<boost::shared_array<double> > ys;
		set<boost::shared_ptr<TGraph> > graphs;

		// Plot
		boost::shared_ptr<TCanvas> tC(new TCanvas("c","", 0,0,700,700));
		gStyle->SetPalette(1);
		size_t color = 1;
		//tC->SetLogy(true);

		for (auto it = isos.begin(); it != isos.end(); ++it, ++color)
		{
			size_t nthetas = (*it)->size();
			boost::shared_array<double> dt(new double[nthetas]),
				P11(new double[nthetas]);
			size_t i = 0;
			for (auto ot = (*it)->begin(); ot != (*it)->end(); ++ot, ++i)
			{
				// Pf here is unnormalized. Normalization invlves integration 
				// over all possible components
				dt[i] = ot->first;
				P11[i] = ot->second.get(2,0,0);
			}

			boost::shared_ptr<TGraph> g(new TGraph(nthetas,dt.get(),P11.get()));
			graphs.insert(g);

			if (it == isos.begin())
			{
				g->Draw("AC");
				g->SetTitle("Isotropic P11");
				g->GetXaxis()->SetTitle("d#Theta");
				g->GetXaxis()->CenterTitle();
				g->GetYaxis()->SetTitle("P11");
				g->GetYaxis()->CenterTitle();
			} else {
				g->Draw("C");
			}
			g->SetLineColor(color);
		}

		// Write the plot
		tC->SaveAs(ofname.c_str());
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

