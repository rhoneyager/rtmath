/* orientation-weights
 * This program will calculate the weights for different orientations of a ddOutput object or 
 * a set of ddPar objects.
*/
#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#pragma warning( pop ) 

#include <Voro++/voro++.hh>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/error/debug.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	const double pi = boost::math::constants::pi<double>();

	try {
		cerr << "rtmath-orientation-weights\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);
		//p.add("output", 2);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		//Ryan_Serialization::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >(), "input ddPar file")
			("betas,b", po::value<vector<string> >(), "Specify beta rotations")
			("thetas,t", po::value<vector<string> > (), "Specify theta rotations")
			("phis,p", po::value<vector<string> >(), "Specify phi rotations")
			("lincosphi", po::value<bool>()->default_value(true), 
			"Treat the theta rotations as linear in cos(theta)")
			("output,o", po::value<string>(), "Output weight file")
			("draw-gnuplot-particles", po::value<string>(), "Write gnuplot particles")
			("draw-gnuplot-cells", po::value<string>(), "Write gnuplot cells")
			("draw-povray-particles", po::value<string>(), "Write povray particles")
			("draw-povray-cells", po::value<string>(), "Write povray cells")
			("bMin", po::value<double>()->default_value(0), "Min beta rotation")
			("tMin", po::value<double>()->default_value(0), "Min theta rotation")
			("pMin", po::value<double>()->default_value(0), "Min phi rotation")
			("bMax", po::value<double>()->default_value(360), "Max beta rotation")
			("tMax", po::value<double>()->default_value(180), "Max theta rotation")
			("pMax", po::value<double>()->default_value(360), "Max phi rotation")
			("nb", po::value<int>()->default_value(6), "Number of beta blocks")
			("nt", po::value<int>()->default_value(6), "Number of theta blocks")
			("np", po::value<int>()->default_value(6), "Number of phi blocks")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << endl;
			cerr << message << endl;
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		double bMin = vm["bMin"].as<double>();
		double tMin = vm["tMin"].as<double>();
		double pMin = vm["pMin"].as<double>();
		double bMax = vm["bMax"].as<double>();
		double tMax = vm["tMax"].as<double>();
		double pMax = vm["pMax"].as<double>();
		int nb = vm["nb"].as<int>();
		int nt = vm["nt"].as<int>();
		int np = vm["np"].as<int>();

		//Ryan_Serialization::process_static_options(vm);
		//ddscat::shapeFileStats::process_static_options(vm);

		std::vector<std::string> sbetas, sthetas, sphis;
		std::vector<std::set<double> > vbetas, vthetas, vphis;

		if (vm.count("betas"))
			sbetas = vm["betas"].as<std::vector<std::string> >();
		if (vm.count("thetas"))
			sthetas = vm["thetas"].as<std::vector<std::string> >();
		if (vm.count("phis"))
			sphis = vm["phis"].as<std::vector<std::string> >();

		const std::string sBadRots = "When manually specifying rotations, the number of "
			"beta, theta and phi rotation tuples must be the same";
		if (sbetas.size() != sthetas.size()) doHelp(sBadRots);
		if (sbetas.size() != sphis.size()) doHelp(sBadRots);
		if (sthetas.size() != sphis.size()) doHelp(sBadRots);

		auto addToVec = [&](const std::vector<std::string>& vstr,
			std::vector<std::set<double> > &vset)
		{
			for (auto &sb : vstr)
			{
				std::set<double> pss;
				paramSet<double> p(sb);
				p.getLong(pss);
				vset.push_back(std::move(pss));
			}
		};

		addToVec(sbetas, vbetas);
		addToVec(sthetas, vthetas);
		addToVec(sphis, vphis);
		
		// Also load in any ddscat.par files for recombination
		std::vector<std::string> parfiles;
		if (vm.count("input"))
			parfiles = vm["input"].as<std::vector<std::string> >();
		for (auto &file : parfiles)
		{
			rtmath::ddscat::ddPar par(file);
			rtmath::ddscat::rotations rot;
			par.getRots(rot);
			std::set<double> b, t, p;
			rot.betas(b);
			rot.thetas(t);
			rot.phis(p);
			vbetas.push_back(std::move(b));
			vthetas.push_back(std::move(t));
			vphis.push_back(std::move(p));
		}


		// Finally, create the rotation tuples
		std::map<boost::tuple<double,double,double>, size_t> rots;
		for (auto &b : vbetas)
			for (auto &t : vthetas)
				for (auto &p : vphis)
				{
					rtmath::ddscat::rotations::populateRotations(b,t,p,rots);
				}


		// Construct the voronoi diagram
		bool lincosphi = vm["lincosphi"].as<bool>();
		if (lincosphi)
		{
			tMin = -1.; //cos(pi*tMin/180.);
			tMax = 1.; //cos(pi*tMax/180.);
		}
		voro::container con(bMin, bMax, tMin, tMax, pMin, pMax, nb, nt, np,
			true, true, true, 8);
		int id = 0;
		std::cout << "Constructing Voronoi diagram\n";
		for (auto &rot : rots)
		{
			double th = rot.first.get<1>();
			double t = th;
			if (lincosphi) th = cos(pi*th/180.);
			double b = rot.first.get<0>();
			double p = rot.first.get<2>();
			std::cout << id << " " << b << "\t" << t << "\t" << th << "\t" << p << std::endl;
			con.put(id,b,th,p);
			++id;
		}

		con.compute_all_cells();
		double cvol = abs((bMax-bMin)*(tMax-tMin)*(pMax-pMin));
		double vvol=con.sum_cell_volumes();
		std::cout << "Container volume: " << cvol
			<< "\nVoroni volume: " << vvol
			<< "\tDifference: " << vvol-cvol 
			<< "\tFraction: " << cvol/vvol << std::endl;

		if (vm.count("output"))
		{
			std::string sofile = vm["output"].as<string>();
			ofstream out(sofile.c_str());
			out << "Beta\tTheta\tPhi\tVolume\tRaw_Weight\tDegeneracy\tWeight\n";

			voro::c_loop_all cl(con);
			voro::voronoicell_neighbor c;
			double b, t, p, v, rw, dw;
			if(cl.start()) do if(con.compute_cell(c,cl)) {
				cl.pos(b,t,p);
				//if (lincosphi) t = acos(t);
				v = c.volume();
				// Raw weight can be determined because the sum of all cell volumes is known
				rw = v / vvol;

				// Find degeneracy
				double degen = 0;
				std::map<boost::tuple<double,double,double>, size_t>::const_iterator it;
				it = std::find_if(rots.begin(), rots.end(),
					[pi,b,t,p,lincosphi](std::pair<const boost::tuple<double,double,double>, size_t> &v) -> bool
				{
					// If overall distance is less than 1e-6, then this is the 
					// same point.
					double ct = v.first.get<1>();
					if (lincosphi) ct = cos(pi*ct/180.);
					double dsq = pow(b - v.first.get<0>(), 2.)
						+ pow(t - ct, 2.)
						+ pow(p - v.first.get<2>(), 2.);
					if (dsq < 1.e-3) return true;
					return false;
				});
				if (it == rots.end())
				{
					std::cerr << "Iteration error " << b << " " << t << " " << p << std::endl;
					continue;
				}

				degen = (double) it->second;
				dw = rw / degen;
				
				if (lincosphi)
				{
					t = acos(t)*180./pi;
				}
				out << b << "\t" << t << "\t" << p << "\t" << v << "\t" << rw
					<< "\t" << degen << "\t" << dw << std::endl;
			} while (cl.inc());
		}


		if (vm.count("draw-gnuplot-particles"))
			con.draw_particles(vm["draw-gnuplot-particles"].as<string>().c_str());
		if (vm.count("draw-gnuplot-cells"))
			con.draw_cells_gnuplot(vm["draw-gnuplot-cells"].as<string>().c_str());
		if (vm.count("draw-povray-particles"))
			con.draw_particles_pov(vm["draw-povray-particles"].as<string>().c_str());
		if (vm.count("draw-povray-cells"))
			con.draw_cells_pov(vm["draw-povray-cells"].as<string>().c_str());

	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

