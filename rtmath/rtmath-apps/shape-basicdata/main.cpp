#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN
#pragma warning( disable : 4068 ) // warning on unknown pragmas - GCC
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

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
#include <boost/shared_array.hpp>
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

#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/shapestatsRotated.h"
#include "../../rtmath/rtmath/ddscat/shapestatsviews.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/serialization.h"
#include "../../rtmath/rtmath/Serialization/shapestats_serialization.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#pragma GCC diagnostic pop

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-basicdata\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input shape files")

			("dipole-spacing,d", po::value<double>(), "Specify dipole spacing (um)")
			("density", "Output lattice cell density")
			("mass", "Output lattice cell mass (mg)")
			("diameter,D", "Calculate max diameter (in physical units is possible)")
			("aeff-filled", "Calculate effective radius with just shape.dat unit cells")
			("aeff-circumsphere", "Calculate effective radius assuming circumscribing sphere")
			("aeff-V-convex-hull", "Calculate effective radius assuming circumscribing sphere volume")
			("aeff-SA-convex-hull", "Calculate effective radius assuming circumscribing sphere surface area")
			("aeff-V-ellipsoid-max", "Calculate effective radius assuming maximum circumscribing ellipsoid volume")

			("aspect-ratios", "Tabulate the aspect ratios calculated for the flake")
			("f-circum-sphere", "Calculate volume fraction with circumscribing sphere")
			("f-convex-hull", "Calculate volume fraction with convex hull")
			("f-ellipsoid-max", "Calculate volume fraction with convex hull")

			("PE", "Plot potential energy (in physical units if possible)")

			("convex-hull","Output convex hull information and vtk file")
			("concave-hull", po::value<string>(), "Output concave hull information and vtk file for given value(s)")
			("output-hull-points", "When hull is calculated, output the points to a csv file")

			("dipole-density-distance", po::value< string >(), 
				"Make histogram and vtk file of number of neighbors within specified spacings")
			("dipole-density-numneighbors", po::value< string >(),
				"Make histogram and vtk file of rms distance to specified nearest neighbors")
			("radial-distribution", po::value< string >(), "Make histogram of the radial distribution function of the dipoles."
				" Pass values specifying number of bins.")
			("concave", "Perform plotting against concave hull")
			//("radial-distribution-scaled", "Scale radial distribution plots according to shell surface area.")
			("fit-peaks", "For selescted plots, attempt to fit peaks")

			("betas,b", po::value<string>()->default_value("0"), "Specify beta rotations")
			("thetas,t", po::value<string>()->default_value("0"), "Specify theta rotations")
			("phis,p", po::value<string>()->default_value("0"), "Specify phi rotations")
			;

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		double dSpacing = 0;
		if (vm.count("dipole-spacing")) 
			dSpacing = vm["dipole-spacing"].as<double>();

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

			boost::shared_ptr<rtmath::ddscat::shapeFileStatsDipoleView> sstatsView
				(new rtmath::ddscat::shapeFileStatsDipoleView(sstats, dSpacing));

			if (vm.count("diameter"))
			{
				cout << "Diameter (d): " << sstats->max_distance << endl;
				if (dSpacing)
					cout << "Diameter (um): " << sstatsView->max_distance() << endl;
			}

			if (vm.count("aeff-filled"))
			{
				cout << "aeff-filled (d): " << sstats->aeff_dipoles_const << endl;
				if (dSpacing)
					cout << "aeff-filled (um): " << sstatsView->aeff_dipoles_const() << endl;
			}

			if (vm.count("aeff-circumsphere"))
			{
				cout << "aeff-circumsphere (d): " << sstats->a_circum_sphere << endl;
				if (dSpacing)
					cout << "aeff-circumsphere (um): " << sstatsView->a_circum_sphere() << endl;
			}

			if (vm.count("aeff-V-convex-hull"))
			{
				cout << "aeff-V-convex-hull (d): " << sstats->aeff_V_convex_hull << endl;
				if (dSpacing)
					cout << "aeff-V-convex-hull (um): " << sstatsView->aeff_V_convex_hull() << endl;
			}

			if (vm.count("aeff-SA-convex-hull"))
			{
				cout << "aeff-SA-convex-hull (d): " << sstats->aeff_SA_convex_hull << endl;
				if (dSpacing)
					cout << "aeff-SA-convex-hull (um): " << sstatsView->aeff_SA_convex_hull() << endl;
			}

			if (vm.count("aeff-V-ellipsoid-max"))
			{
				cout << "aeff-V-ellipsoid-max (d): " << sstats->aeff_ellipsoid_max << endl;
				if (dSpacing)
					cout << "aeff-V-ellipsoid-max (um): " << sstatsView->aeff_ellipsoid_max() << endl;
			}

			if (vm.count("aspect-ratios"))
			{
				cout << "Aspect ratios:" << endl;
				cout << "\tAxy\tAxz\tAyz\n";
				auto rt = sstats->rotations.begin();
				cout << "Abs\t" 
					<< (*rt)->as_abs(0,1) << "\t" 
					<< (*rt)->as_abs(0,2) << "\t" 
					<< (*rt)->as_abs(1,2) << endl;
				cout << "Amean\t"
					<< (*rt)->as_abs_mean(0,1) << "\t" 
					<< (*rt)->as_abs_mean(0,2) << "\t" 
					<< (*rt)->as_abs_mean(1,2) << endl;
				cout << "RMS\t"
					<< (*rt)->as_rms(0,1) << "\t" 
					<< (*rt)->as_rms(0,2) << "\t" 
					<< (*rt)->as_rms(1,2) << endl;
				cout << endl;
			}

			if (vm.count("f-circum-sphere"))
			{
				cout << "f-circum-sphere: " << sstats->f_circum_sphere << endl;
			}

			if (vm.count("f-convex-hull"))
			{
				cout << "f-convex-hull: " << sstats->f_convex_hull << endl;
			}

			if (vm.count("f-ellipsoid-max"))
			{
				cout << "f-ellipsoid-max: " << sstats->f_ellipsoid_max << endl;
			}

			if (vm.count("convex-hull"))
			{
				if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->filename.c_str());
				string ofname = *it;
				ofname.append("-convex-hull.vtk");
				rtmath::ddscat::convexHull cvx(*(sstats->_shp->_pclObj->cloud));
				cvx.constructHull();
				cvx.writeVTKhull(ofname);
				cout << "Hull has volume: " << cvx.volume() << endl;
				cout << "Hull has surface area: " << cvx.surfaceArea() << endl;
				//cout << "Hull has " << cvx._nFaces << " faces." << endl;

				if (vm.count("output-hull-points"))
					{
						ostringstream fvname;
						fvname << *it << "-convex-hull-points.csv";
						ofstream fout(fvname.str().c_str());
						fout << fvname.str() << endl;
						fout << "x,y,z,r\n";
						for (auto pt = cvx._hullPts.begin(); pt != cvx._hullPts.end(); ++pt)
						{
							double s = (dSpacing) ? dSpacing : 1;
							fout << s*pt->x << "," << s*pt->y << "," << s*pt->z << ","
								<< sqrt( pow(pt->x *s,2.0) + pow(pt->y *s,2.0) + pow(pt->z *s,2.0) )
								<< "\n";
						}
						fout.close();
					}
			}

			if (vm.count("concave-hull"))
			{
				string salphas = vm["concave-hull"].as<string>();
				paramSet<double> chulls(salphas);
				for (auto ot = chulls.begin(); ot != chulls.end(); ot++)
				{
					if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->filename.c_str());
					ostringstream fname;
					fname << *it << "-concave-hull-" << *ot << ".vtk";
					rtmath::ddscat::concaveHull ccv(*(sstats->_shp->_pclObj->cloud));
					ccv.constructHull(*ot);
					ccv.writeVTKhull(fname.str());

					if (vm.count("output-hull-points"))
					{
						ostringstream fvname;
						fvname << *it << "-concave-hull-" << *ot << "-points.csv";
						ofstream fout(fvname.str().c_str());
						fout << fvname.str() << endl;
						fout << "x,y,z,r\n";
						for (auto pt = ccv._hullPts.begin(); pt != ccv._hullPts.end(); ++pt)
						{
							double s = (dSpacing) ? dSpacing : 1;
							fout << s*pt->x << "," << s*pt->y << "," << s*pt->z << ","
								<< s*sqrt( pow(pt->x,2.0f) + pow(pt->y,2.0f) + pow(pt->z,2.0f) )
								<< "\n";
						}
						fout.close();
					}
					/*
					cout << "Hull has volume: " << ccv._volume << endl;
					cout << "Hull has surface area: " << ccv._surfarea << endl;
					cout << "Hull has " << ccv._nFaces << " faces." << endl;
					*/
				}
			}

			/*	("dipole-density-distance", po::value< vector<string> >(), 
					"Make histogram and vtk file of number of neighbors within specified spacings")
				("dipole-density-numneighbors", po::value< vector<string> >(),
					"Make histogram and vtk file of rms distance to specified nearest neighbors")
				*/
			if (vm.count("radial-distribution"))
			{
				const double lambda = 2.0; // In case of concave hull
				string snumbins = vm["radial-distribution"].as<string>();
				paramSet<size_t> numbins(snumbins)
				if (!sstats->load()) throw rtmath::debug::xMissingFile(sstats->_shp->_filename.c_str());
				if (!dSpacing) throw rtmath::debug::xBadInput("Need dipole spacings for radial distribution plot");
				boost::shared_ptr<rtmath::ddscat::shapeFileStatsRotatedView> sstatsRView
					(new rtmath::ddscat::shapeFileStatsRotatedView(sstats->calcStatsRot(0,0,0), dSpacing));

				for (auto ot = numbins.begin(); ot != numbins.end(); ++ot)
				{
					ostringstream ofname;
					ofname << *it << "-radial-distribution-" << *ot << ".png";
					string fbase = ofname.str();

					// Get reference to the sstats _shp pointContainer
					pcl::PointCloud<pcl::PointXYZ>::Ptr pc = sstats->_shp->_pclObj->cloud;

					rtmath::ddscat::concaveHull ccv(*(sstats->_shp->_pclObj->cloud));
					if (vm.count("concave"))
					{
						ccv.constructHull(lambda);
						// Yes, I'm doing a deep copy for now. Will rewrite hull correctly to use Ptrs later.
						pcl::PointCloud<pcl::PointXYZ>::Ptr ma(ccv._hullPts.makeShared());
						pc = ma;
					}

					
					boost::shared_ptr<TCanvas> tC(new TCanvas("c","Radial distribution plots", 0, 0, 2100, 600));
					boost::shared_ptr<TH1F> tR(new TH1F());

					double minr = 0;
					double maxr = sstatsRView->max().get(2,3,0);

					double Nrange = maxr - minr;
					double maxRange = maxr + (Nrange/10);
					tR->SetBins(*ot, 0, maxRange); // ot in # bins,

					int ne = pc->size();
					boost::shared_array<Double_t> par(new Double_t[ (Int_t) (maxRange*3) ]);
					boost::shared_array<Double_t> w(new Double_t[ne]); // Is this the correct size?
					std::fill_n(par.get(),(Int_t) (maxRange*3), 0);
					std::fill_n(w.get(), ne, 1);

					for(auto pt = pc->begin(); pt != pc->end(); ++pt)
					{
						double i = sqrt( pow(pt->x *dSpacing,2.0) + pow(pt->y *dSpacing,2.0) + pow(pt->z *dSpacing,2.0) );
						double w = 1;
						tR->Fill(i,w);
					}

					gStyle->SetPalette(1);
					//tC->Divide(1,1);
					//tC->cd(1);
					tR->Draw();
					tR->SetTitle(string("Radial distribution of ").append(*it).c_str());
					tR->GetXaxis()->SetTitle("Radial distance (um)");
					
					tR->GetXaxis()->CenterTitle();
					tR->GetYaxis()->SetTitle("Frequency");
					tR->GetYaxis()->CenterTitle();

					// These definitions are here to prevent the pointers from being released.
					boost::shared_ptr<TH1F> h2;
					boost::shared_ptr<TSpectrum> s;
					TH1 *hb;
					boost::shared_ptr<TF1> fline;
					boost::shared_ptr<TF1> fit;
					if (vm.count("fit-peaks"))
					{
						Int_t npeaks = 20; // Match a max of 20 peaks
						// Function used for radial distribution peak-fitting plots
						auto fpeaks = [&](Double_t *x, Double_t *par) -> Double_t
						{
							Double_t result = par[0] + par[1]*x[0];
							for (Int_t p=0;p<npeaks;p++)
							{
								Double_t norm = par[3*p+2];
								Double_t mean = par[3*p+3];
								Double_t sigma = par[3*p+4];
								result += norm*TMath::Gaus(x[0],mean,sigma);
							}
							return result;
						};

						h2 = boost::shared_ptr<TH1F> ((TH1F*)tR->Clone("h2"));

						// Find the peaks and background

						s = boost::shared_ptr<TSpectrum> (new TSpectrum(2*npeaks));
						Int_t nfound = s->Search(tR.get(),3,"",0.1);
						cerr << "Found " << nfound << " candidate peaks to fit\n";
						hb = s->Background(tR.get(),20,"same");
						//hb->Draw("same");
						//if (hb) tC->Update();

						// Fit everything

						fline = boost::shared_ptr<TF1> (new TF1("fline","pol1",0,maxRange));
						tR->Fit("fline","qn");
						par[0] = fline->GetParameter(0);
						par[1] = fline->GetParameter(1);
						npeaks = 0;
						Float_t *xpeaks = s->GetPositionX();
						for (Int_t p=0;p<nfound;p++)
						{
							Float_t xp = xpeaks[p];
							Int_t bin = tR->GetXaxis()->FindBin(xp);
							Float_t yp = (Float_t) tR->GetBinContent(bin);
							if (yp-TMath::Sqrt(yp) < fline->Eval(xp)) continue;
							par[3*npeaks+2] = yp;
							par[3*npeaks+3] = xp;
							par[3*npeaks+4] = 150;
							npeaks++;
						}

						cerr << "Found " << npeaks << " useful peaks to fit\n";
						fit = boost::shared_ptr<TF1> (new TF1("fit",fpeaks,0,maxRange,2+3*npeaks));
						TVirtualFitter::Fitter(h2.get(),10+3*npeaks);
						fit->SetParameters(par.get());
						fit->SetNpx(2000); // Set number of points used to draw function
						h2->Fit("fit");
						//hb->Draw("same");
						//tC->Update();
						cerr << endl;
						for (Int_t p=0; p< maxRange*3; p++)
						{
							cerr << p << "\t" << par[p] << endl;
						}
						cerr << endl;
					}


					tC->SaveAs(fbase.c_str());
				}
			}

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
					const double beta = (*ot)->beta;
					if (beta != 0) continue;
					const double theta = (*ot)->theta;
					const double phi = (*ot)->phi;
					const double PEx = (*ot)->PE[0].get(2,0,0);
					const double PEy = (*ot)->PE[0].get(2,1,0);
					const double PEz = (*ot)->PE[0].get(2,2,0);
					
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

