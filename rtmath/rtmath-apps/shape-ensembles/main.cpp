#pragma warning( push )
#pragma warning( disable : 4996 )
#pragma warning( disable : 4800 )
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_array.hpp>


#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/density.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/MagickLINK.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#pragma warning( pop ) 
int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-ensembles\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files or shape statistics")

			("frequency,f", po::value<double>()->default_value(94), "Specify frequency for analysis (GHz)")
			("temperature,T",po::value<double>()->default_value(263),"Specify temperature for analysis (K)")
			("dipole-spacing,d", po::value<double>()->default_value(40), "Specify dipole spacing (um)")

			("mass-aeff", "Plot masses of flakes vs their effective radius")
			("N-f", "Scatter plot of number of dipoles vs several volume fractions")
			("aeff-f", "Scatter plot of effective radius of filled cells vs volume fractions")
			("aspect-ratios", "Scatter plot of aspect ratios Axy and Axz")
			("Vplots", "Comparison scatter plots of different volume rendering mechanisms")
			("orientation-PDF-theta", po::value< vector<string> >(), 
			"Comparative PDFs for particle orientation in theta for given total energy")

			("output-prefix,o", po::value<string>()->default_value("ensemble"), "Specify output filename prefix for plots");

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		string outprefix;
		if (vm.count("output-prefix"))
			outprefix = vm["output-prefix"].as<string>();

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

		// Validate input files
		for (auto it = inputs.begin(); it != inputs.end(); it++)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path pt = pi / "target.out";
				path ps = pi / "shape.dat";
				if (exists(pt)) *it = pt.string();
				else if (exists(ps)) *it = ps.string();
				else throw rtmath::debug::xPathExistsWrongType(it->c_str());
			}
		}

		// Specify beta, theta, phi rotations
		string sbeta = "0";
		string stheta = "0";
		string sphi = "0";

		paramSet<double> betas(sbeta);
		paramSet<double> thetas(stheta);
		paramSet<double> phis(sphi);

		double temp = vm["temperature"].as<double>();
		double freq = vm["frequency"].as<double>();
		double d = 0;
		if (vm.count("dipole-spacing"))
			d = vm["dipole-spacing"].as<double>();

		// Figure out the density of ice from the data
		double iceden = rtmath::density::ice1h(temp) * 1.e-12; // in g/um^3
		vector<double> densities(2,iceden), masses(2,iceden);
		// Conveniently the same with a unit cell. TODO: allow for others.

		vector<boost::shared_ptr<rtmath::ddscat::shapeFileStats> > Stats;
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
				for (auto beta = betas.begin(); beta != betas.end(); beta++)
				{
					for (auto theta = thetas.begin(); theta != thetas.end(); theta++)
					{
						for (auto phi = phis.begin(); phi != phis.end(); phi++)
						{
							cerr << "\tCalculating rotation (beta,theta,phi): ("
								<< *beta << ", " << *theta << ", " << *phi << ")" << endl;
							sstats->calcStatsRot(*beta,*theta,*phi);
						}
					}
				}
			} else {
				cerr << "\tStats file detected\n";
				rtmath::serialization::read<rtmath::ddscat::shapeFileStats>
					(*sstats, *it);
			}
			Stats.push_back(sstats);
		}

		// All stats entries are now loaded. 
		// Now to make the appropriate plots.
		
		//if (vm.count("mass-aeff"))
		{
			//if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
			string ofname = outprefix;
			ofname.append("-mass-aeff.png");

			const size_t nP = Stats.size();
			boost::scoped_array<double> aeffs(new double[nP]);
			boost::scoped_array<double> masses(new double[nP]);

			boost::shared_ptr<TCanvas> tC(new TCanvas("c","PE plot", 0,0,700,700));
			

			// Collect all masses
			size_t i=0;
			for (auto it = Stats.begin(); it != Stats.end(); ++it, i++)
			{
				rtmath::ddscat::shapeFileStatsDipoleView viewBase(*it, d);
				

				double Vcells = viewBase.V_dipoles_const();
				double mass = Vcells * densities[1];

				double aeff = viewBase.aeff_dipoles_const(); //(*it)->aeff_dipoles_const;

				aeffs[i] = aeff;
				masses[i] = mass * 1.e3;
				//tM->Fill(aeff,mass);
				//tM->SetPoint(i,mass,aeff);
			}
			boost::shared_ptr<TGraph> tM(new TGraph(nP,aeffs.get(),masses.get()));

			gStyle->SetPalette(1);
			tM->Draw("A*");
			tM->SetTitle("Mass vs. Effective Radius");
			tM->GetXaxis()->SetTitle("Effective radius (um)");
			tM->GetXaxis()->CenterTitle();
			tM->GetYaxis()->SetTitle("Mass (mg)");
			tM->GetYaxis()->CenterTitle();

			tC->SaveAs(ofname.c_str());
		}

		//if (vm.count("aspect-ratios")) // todo: give a distinct name
		{
			//if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
			string ofname = outprefix;
			ofname.append("-aspect-ratios-abs.png");

			const size_t nP = Stats.size();
			boost::scoped_array<double> axys(new double[nP]);
			boost::scoped_array<double> axzs(new double[nP]);

			boost::shared_ptr<TCanvas> tC(new TCanvas("c2","", 0,0,700,700));
			
			size_t i=0;
			for (auto it = Stats.begin(); it != Stats.end(); ++it, i++)
			{
				auto rn = (*it)->rotations.begin();

				double axy = (*rn)->as_abs.get(2,1,0);
				double axz = (*rn)->as_abs.get(2,2,0);

				axys[i] = axy;
				axzs[i] = axz;
				//tM->Fill(aeff,mass);
				//tM->SetPoint(i,mass,aeff);
			}
			boost::shared_ptr<TGraph> tM(new TGraph(nP,axys.get(),axzs.get()));

			gStyle->SetPalette(1);
			tM->Draw("A*");
			tM->SetTitle("Absolute Aspect Ratio Relations");
			tM->GetXaxis()->SetTitle("Axy");
			tM->GetXaxis()->CenterTitle();
			tM->GetYaxis()->SetTitle("Axz");
			tM->GetYaxis()->CenterTitle();

			tC->SaveAs(ofname.c_str());
		}

		//if (vm.count("aspect-ratios")) // todo: give a new name
		{
			//if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
			string ofname = outprefix;
			ofname.append("-aspect-ratios-rms.png");

			const size_t nP = Stats.size();
			boost::scoped_array<double> axys(new double[nP]);
			boost::scoped_array<double> axzs(new double[nP]);

			boost::shared_ptr<TCanvas> tC(new TCanvas("c2","", 0,0,700,700));
			
			size_t i=0;
			for (auto it = Stats.begin(); it != Stats.end(); ++it, i++)
			{
				auto rn = (*it)->rotations.begin();

				double axy = (*rn)->as_rms.get(2,1,0);
				double axz = (*rn)->as_rms.get(2,2,0);

				axys[i] = axy;
				axzs[i] = axz;
				//tM->Fill(aeff,mass);
				//tM->SetPoint(i,mass,aeff);
			}
			boost::shared_ptr<TGraph> tM(new TGraph(nP,axys.get(),axzs.get()));

			gStyle->SetPalette(1);
			tM->Draw("A*");
			tM->SetTitle("RMS Aspect Ratio Relations");
			tM->GetXaxis()->SetTitle("Axy");
			tM->GetXaxis()->CenterTitle();
			tM->GetYaxis()->SetTitle("Axz");
			tM->GetYaxis()->CenterTitle();

			tC->SaveAs(ofname.c_str());
		}

		//if (vm.count("aeff-f"))
		{
			//if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
			string ofname = outprefix;
			ofname.append("-aeff-f.png");

			const size_t nP = Stats.size();
			boost::scoped_array<double> aeffs(new double[nP]);
			boost::scoped_array<double> fspheres(new double[nP]);
			boost::scoped_array<double> fconvexs(new double[nP]);

			boost::shared_ptr<TCanvas> tC(new TCanvas("c2","", 0,0,700,700));
			
			size_t i=0;

			double min_x = 0, max_x = 0, min_y = 0, max_y = 0;

			for (auto it = Stats.begin(); it != Stats.end(); ++it, i++)
			{
				auto rn = (*it)->rotations.begin();

				double axy = (*rn)->as_abs.get(2,1,0);
				double axz = (*rn)->as_abs.get(2,2,0);

				rtmath::ddscat::shapeFileStatsDipoleView viewBase(*it, d);
				
				double aeff = viewBase.aeff_dipoles_const();
				double fCircum = viewBase.f_circum_sphere();
				double fConvex = viewBase.f_convex_hull();

				if (i==0)
				{
					min_x = aeff;
					max_x = aeff;
					min_y = fCircum;
					max_y = fCircum;
				}

				if (aeff < min_x) min_x = aeff;
				if (aeff > max_x) max_x = aeff;
				if(fCircum < min_y) min_y = fCircum;
				if(fConvex < min_y) min_y = fConvex;
				if(fCircum > max_y) max_y = fCircum;
				if(fConvex > max_y) max_y = fConvex;

				aeffs[i] = aeff;
				fspheres[i] = fCircum;
				fconvexs[i] = fConvex;
			}

			boost::shared_ptr<TGraph> ts(new TGraph(nP,aeffs.get(),fspheres.get()));
			boost::shared_ptr<TGraph> tc(new TGraph(nP,aeffs.get(),fconvexs.get()));

			gStyle->SetPalette(1);
			ts->Draw("A*");
			ts->SetTitle("Volume fraction relations");
			ts->GetXaxis()->SetTitle("aeff");
			ts->GetXaxis()->CenterTitle();
			ts->GetYaxis()->SetTitle("f");
			ts->GetYaxis()->CenterTitle();
			ts->GetXaxis()->SetLimits(min_x,max_x);
			ts->GetYaxis()->SetLimits(min_y,max_y);

			tc->Draw("*");
			tc->SetMarkerColor(2);


			tC->SaveAs(ofname.c_str());
		}

		if (vm.count("PE"))
		{
			/*
			string ofname = *it;
			ofname.append("-PE.png");

			// Extract the potential energies against x-dimension, 
			// using theta and phi as the axes
			const size_t np = sstats->rotations.size();
			boost::shared_ptr<TCanvas> tC(new TCanvas("c","PE plot", 0,0,2100,600));
			boost::shared_ptr<TGraph2D> tPEx(new TGraph2D());
			boost::shared_ptr<TGraph2D> tPEy(new TGraph2D());
			boost::shared_ptr<TGraph2D> tPEz(new TGraph2D());

			size_t n=0;
			// TODO: resizing only for beta = 0
			for (auto ot = sstats->rotations.begin(); ot != sstats->rotations.end(); ot++, n++)
			{
				const double beta = ot->beta;
				if (beta != 0) continue;
				const double theta = ot->theta;
				const double phi = ot->phi;
				const double PEx = ot->PE[0].get(2,0,0);
				const double PEy = ot->PE[0].get(2,1,0);
				const double PEz = ot->PE[0].get(2,2,0);

				tPEx->SetPoint(n,theta,phi,PEx);
				tPEy->SetPoint(n,theta,phi,PEy);
				tPEz->SetPoint(n,theta,phi,PEz);
			}
			gStyle->SetPalette(1);
			tC->Divide(3,1);

			tC->cd(1);
			tPEx->Draw("surf3");
			tPEx->SetTitle("Potential Energy - x");
			tPEx->GetXaxis()->SetTitle("#theta");
			tPEx->GetXaxis()->CenterTitle();
			tPEx->GetYaxis()->SetTitle("#phi");
			tPEx->GetYaxis()->CenterTitle();
			tPEx->GetZaxis()->SetTitle("PE (scaled)");
			tPEx->GetZaxis()->CenterTitle();


			tC->cd(2);
			tPEy->Draw("surf3");
			tPEy->SetTitle("Potential Energy - y");
			tPEy->GetXaxis()->SetTitle("#theta");
			tPEy->GetXaxis()->CenterTitle();
			tPEy->GetYaxis()->SetTitle("#phi");
			tPEy->GetYaxis()->CenterTitle();
			tPEy->GetZaxis()->SetTitle("PE (scaled)");
			tPEy->GetZaxis()->CenterTitle();

			tC->cd(3);
			tPEz->Draw("surf3");
			tPEz->SetTitle("Potential Energy - z");
			tPEz->GetXaxis()->SetTitle("#theta");
			tPEz->GetXaxis()->CenterTitle();
			tPEz->GetYaxis()->SetTitle("#phi");
			tPEz->GetYaxis()->CenterTitle();
			tPEz->GetZaxis()->SetTitle("PE (scaled)");
			tPEz->GetZaxis()->CenterTitle();

			tC->SaveAs(ofname.c_str());
			*/
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

