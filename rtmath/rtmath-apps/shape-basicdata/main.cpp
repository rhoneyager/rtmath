#pragma warning( push )
#pragma warning( disable : 4996 )
#pragma warning( disable : 4800 )
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN

#include "../../rtmath/rtmath/ROOTlink.h"
#include "../../rtmath/rtmath/VTKlink.h"

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

#pragma warning( pop ) 
#include "../../rtmath/rtmath/matrixop.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/MagickLINK.h"
#include "../../rtmath/rtmath/Garrett/pclstuff.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/matrixop.h"



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
			/*
			("dipole-spacing,d", po::value<double>(), "Specify dipole spacing (um)")
			("density", po::value<double>(), "Specify lattice cell density (mg/um^3)")
			("mass", po::value<double>(), "Specify lattice cell mass (mg)")
			*/
			("diameter,D", "Calculate max diameter")
			("PE", "Plot potential energy")
			("convex-hull","Output convex hull information and vtk file")
			("concave-hull", po::value<string>(), "Output concave hull information and vtk file for given value(s)")

			("dipole-density-distance", po::value< string >(), 
				"Make histogram and vtk file of number of neighbors within specified spacings")
			("dipole-density-numneighbors", po::value< string >(),
				"Make histogram and vtk file of rms distance to specified nearest neighbors")

			("betas,b", po::value<string>()->default_value("0"), "Specify beta rotations")
			("thetas,t", po::value<string>()->default_value("0:15:90"), "Specify theta rotations")
			("phis,p", po::value<string>()->default_value("0:15:90"), "Specify phi rotations")
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

			if (vm.count("convex-hull"))
			{
				if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
				string ofname = *it;
				ofname.append("-convex-hull.vtk");
				rtmath::ddscat::convexHull cvx(sstats->_shp->_latticePtsStd);
				cvx.constructHull();
				cvx.writeVTKhull(ofname);
				cout << "Hull has volume: " << cvx._volume << endl;
				cout << "Hull has surface area: " << cvx._surfarea << endl;
				//cout << "Hull has " << cvx._nFaces << " faces." << endl;
			}

			if (vm.count("concave-hull"))
			{
				string salphas = vm["concave-hull"].as<string>();
				paramSet<double> chulls(salphas);
				for (auto ot = chulls.begin(); ot != chulls.end(); ot++)
				{
					if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
					ostringstream fname;
					fname << *it << "-concave-hull-" << *ot << ".vtk";
					rtmath::ddscat::concaveHull ccv(sstats->_shp->_latticePtsStd);
					ccv.constructHull(*ot);
					ccv.writeVTKhull(fname.str());
					/*
					cout << "Hull has volume: " << ccv._volume << endl;
					cout << "Hull has surface area: " << ccv._surfarea << endl;
					cout << "Hull has " << ccv._nFaces << " faces." << endl;
					*/
				}
			}

			if (vm.count("diameter"))
			{
				cout << "Diameter (d): " << sstats->max_distance << endl;
			}

			/*	("dipole-density-distance", po::value< vector<string> >(), 
					"Make histogram and vtk file of number of neighbors within specified spacings")
				("dipole-density-numneighbors", po::value< vector<string> >(),
					"Make histogram and vtk file of rms distance to specified nearest neighbors")
				*/

			if (vm.count("dipole-density-distance") || vm.count("dipole-density-numneighbors"))
			{
				string sdists, sneigh;
				if (vm.count("dipole-density-distance"))
					sdists = vm["dipole-density-distance"].as<string>();
				if (vm.count("dipole-density-numneighbors"))
					sneigh = vm["dipole-density-numneighbors"].as<string>();
				paramSet<float> cdists(sdists);
				paramSet<float> cneigh(sneigh);
				// Construct pairwise containers to handle both cases.
				set<pair<float, bool> > c;
				for (auto ot = cdists.begin(); ot != cdists.end(); ot++)
				{
					c.insert(pair<float,bool>(*ot,false));
				}
				for (auto ot = cneigh.begin(); ot != cneigh.end(); ot++)
				{
					c.insert(pair<float,bool>(*ot,true));
				}

				for (auto ot = c.begin(); ot != c.end(); ot++)
				{
					if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
					ostringstream ofname;
					if (ot->second == false)
						ofname << *it << "-dipole-density-d-" << ot->first;
					else
						ofname << *it << "-dipole-density-neighbors-" << ot->first;
					string fbase = ofname.str();

					// Do lots of kd tree sorting to generate histogram in 1d of number of neighbors within specified radius
					// Select the center slice of the shape and make a numeric 2d histogram reflecting this
					// Also produce 3d vtk file output

					// Get reference to the sstats _shp pointContainer
					boost::shared_ptr<rtmath::Garrett::pointContainer> pc = sstats->_shp->_pclObj;

					// The container for the number of neighbors container and the rms distance container
					//pcl::PointXYZRGB
					pcl::PointCloud<pcl::PointXYZI> nneighbors; // XYZ and intensity (float)
					pcl::PointCloud<pcl::PointXYZI> rmsneighbors;
					nneighbors.reserve(pc->cloud->size());
					rmsneighbors.reserve(pc->cloud->size());
					//pcl::PointXYZI a;
					
					// The set containing the neighbor number information
					multiset<float> sneighbors;
					multiset<float> srms;
					
					// Generate kd trees relative to this node
					pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
					tree->setInputCloud (pc->cloud);

					vtkSmartPointer<vtkDoubleArray> vRMS = 
						vtkSmartPointer<vtkDoubleArray>::New();
					vtkSmartPointer<vtkDoubleArray> vN = 
						vtkSmartPointer<vtkDoubleArray>::New();

					vRMS->SetName("RMS distances");
					vRMS->SetNumberOfValues(pc->cloud->size());
					//vRMS->SetNumberOfComponents(3);
					//vRMS->SetNumberOfTuples(pc->cloud->size());

					vN->SetNumberOfValues(pc->cloud->size());
					if (ot->second == false)
						vN->SetName("Num points within specified distance");
					else
						vN->SetName("Distances of nearest neighbors");
					//vN->SetNumberOfComponents(3);
					//vN->SetNumberOfTuples(pc->cloud->size());

					size_t p=0; // Stupid VTK requirement
					for (auto sp = pc->cloud->begin(); sp != pc->cloud->end(); sp++, p++)
					{
						vector<int> indices;
						vector<float> d_sq;
						//pc->cloud->begin()->x;
						if (ot->second == false)
							tree->radiusSearch(*sp, ot->first, indices, d_sq);
						else
							tree->nearestKSearch(*sp, (int) ot->first, indices, d_sq);
						// d_sq now has all of the squared distances. We also have the number of points in the region.
						valarray<float> vdsq(d_sq.data(),d_sq.size());
						double dmean = 0;
						for_each(d_sq.begin(), d_sq.end(), [&] (float i)
						{
							dmean += sqrt(i)/d_sq.size();
						});

						double drms = sqrt(vdsq.sum() / (float) d_sq.size());

						pcl::PointXYZI n, rms;
						n.x = sp->x;
						n.y = sp->y;
						n.z = sp->z;
						if (ot->second == false)
						{
							n.intensity = d_sq.size();
							sneighbors.insert((float) d_sq.size());
						} else {
							n.intensity = dmean;
							sneighbors.insert((float) dmean);
						}

						rms.x = sp->x;
						rms.y = sp->y;
						rms.z = sp->z;
						rms.intensity = drms;

						// Container insertion
						
						srms.insert(drms);
						vN->SetValue(p,n.intensity);
						vRMS->SetValue(p,rms.intensity);
						nneighbors.push_back(move(n));
						rmsneighbors.push_back(move(rms));
					}

					// All of the points have been processed. Time to write the vtk files and plot the histograms
					// I don't like the pcl conversion defaults, so I get to implement my own!
					vtkSmartPointer<vtkStructuredGrid> vpc = vtkSmartPointer<vtkStructuredGrid>::New();
					pcl::io::pointCloudTovtkStructuredGrid<pcl::PointXYZI>(nneighbors, vpc);
					
					// The structured grid has now has the point coordinates. Now to attach the arrays for coloring
					//vpc->GetPointData().AddArray(vRMS);
					vpc->GetPointData()->AddArray(vRMS);
					vpc->GetPointData()->AddArray(vN);
					
					vtkSmartPointer<vtkXMLStructuredGridWriter> writer =
						vtkSmartPointer<vtkXMLStructuredGridWriter>::New();
					writer->SetFileName(string(fbase).append("-neighbors.vts").c_str());
					writer->SetInputConnection(vpc->GetProducerPort());
					//writer->SetInputData(pcN);
					writer->Write();


					// Plotting two 1d histograms side-by-side - sneighbors and srms

					boost::shared_ptr<TCanvas> tC(new TCanvas("c","Density plots", 0, 0, 2100, 600));
					boost::shared_ptr<TH1F> tNN(new TH1F());
					boost::shared_ptr<TH1F> tRMS(new TH1F());

					double Nrange = *(sneighbors.rbegin()) - *(sneighbors.begin());
					double Rrange = *(srms.rbegin()) - *(srms.begin());
					tNN->SetBins(sneighbors.size(), *(sneighbors.begin()) - (Nrange/10.), *(sneighbors.rbegin()) + (Nrange/10.));
					tRMS->SetBins(srms.size(), *(srms.begin()) - (Rrange/10.), *(srms.rbegin()) + (Rrange/10.));

					for_each(sneighbors.begin(), sneighbors.end(), [&] (float i)
					{
						tNN->Fill(i);
					});
					for_each(srms.begin(), srms.end(), [&] (float i)
					{
						tRMS->Fill(i);
					});

					gStyle->SetPalette(1);
					tC->Divide(2,1);

					tC->cd(1);
					tNN->Draw();
					if (ot->second == false)
					{
						tNN->SetTitle(string("Number of neighbors within distance ").append(boost::lexical_cast<string>(ot->first)).c_str());
						tNN->GetXaxis()->SetTitle("Number of neighbors");
					} else {
						tNN->SetTitle(string("Mean Distances of nearest ").append(boost::lexical_cast<string>(ot->first)).append(" neighbors").c_str());
						tNN->GetXaxis()->SetTitle("Mean neighbor distance");
					}
					
					tNN->GetXaxis()->CenterTitle();
					tNN->GetYaxis()->SetTitle("Frequency");
					tNN->GetYaxis()->CenterTitle();

					tC->cd(2);
					tRMS->Draw();
					if (ot->second == false)
					{
						tRMS->SetTitle(string("RMS distance of neighbors within distance ").append(boost::lexical_cast<string>(ot->first)).c_str());
						tRMS->GetXaxis()->SetTitle("Distance");
					} else {
						tRMS->SetTitle(string("RMS distance of nearest ").append(boost::lexical_cast<string>(ot->first)).append(" neighbors").c_str());
						tRMS->GetXaxis()->SetTitle("Distance");
					}
					
					tRMS->GetXaxis()->CenterTitle();
					tRMS->GetYaxis()->SetTitle("Frequency");
					tRMS->GetYaxis()->CenterTitle();

					tC->SaveAs(string(fbase).append("-histogram1.png").c_str());
				}
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

