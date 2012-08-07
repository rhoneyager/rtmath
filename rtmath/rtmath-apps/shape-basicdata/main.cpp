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
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/error/error.h"
//#include "../../rtmath/rtmath/rtmath.h"
#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/MagickLINK.h"


int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-basicdata\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files")
			("diameter,D", "Calculate max diameter")
			("PE", "Plot potential energy")
			("betas,b", po::value<string>()->default_value("0"), "Specify beta rotations")
			("thetas,t", po::value<string>()->default_value("0:15:90"), "Specify theta rotations")
			("phis,p", po::value<string>()->default_value("0:15:90"), "Specify phi rotations"),
			("vectorstats,V", "Stats file is old format, with stats hidden within a vector.");
//			("output,o", "output filename")

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
		string sbeta = vm["betas"].as<string>();
		string stheta = vm["thetas"].as<string>();
		string sphi = vm["phis"].as<string>();

		paramSet<double> betas(sbeta);
		paramSet<double> thetas(stheta);
		paramSet<double> phis(sphi);

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
				if (!vm.count("separate-outputs"))
				{
					rtmath::serialization::read<rtmath::ddscat::shapeFileStats>
						(*sstats, *it);
				} else {
					// stats are in a vector (old method)
					vector<rtmath::ddscat::shapeFileStats> vs;
					rtmath::serialization::read<vector<rtmath::ddscat::shapeFileStats> >
						(vs, *it);
					if (vs.size())
					{
						*sstats = vs[0];
					} else {
						throw rtmath::debug::xUnknownFileFormat(it->c_str());
					}
				}
				
			}

			//cerr << "\tCalculating rotation (beta,theta,phi): ("
			//	<< *beta << ", " << *theta << ", " << *phi << ")" << endl;
			//sstats.calcStatsRot(*beta,*theta,*phi);
		
			if (vm.count("diameter"))
			{
				cout << "Diameter (d): " << sstats->max_distance << endl;
			}
			if (vm.count("PE"))
			{
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
					const double PEx = ot->PE.get(2,0,0);
					const double PEy = ot->PE.get(2,1,0);
					const double PEz = ot->PE.get(2,2,0);
					
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

