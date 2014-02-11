/* This program is designed to extract tsv data from sets of ddOutput files. */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-output-extract\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >()->multitoken(),"specify input directories or files")
			("output,o", po::value<string>(), "specify output file")
			("force-description", po::value<string>(), "Override the description of each entry")
			("shapes,s", po::value<vector<string> >()->multitoken(), 
			"Specify optional shapefiles for matching against pure avg file input. "
			"Matching s done based on common elements in the filenames.")
			;

		po::positional_options_description p;
		p.add("input",-1);
		//p.add("output",2);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		Ryan_Serialization::process_static_options(vm);
		rtmath::ddscat::ddUtil::process_static_options(vm);
		//rtmath::ddscat::shapeFileStats::process_static_options(vm);
		rtmath::ddscat::ddOutput::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");
		
		vector<string> vsInput, vsShapes;
		string sOutput;
		if (vm.count("input")) vsInput = vm["input"].as<vector<string> >();
		else doHelp("Need to specify input(s)");
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		else doHelp("Need to specify output file.");

		if (vm.count("shapes")) vsShapes = vm["shapes"].as<vector<string> >();

		string sDescrip;
		if (vm.count("force-description")) sDescrip = vm["force-description"].as<string>();

		ofstream out(sOutput.c_str());
		out << "Filename\tDescription\tShape Hash\tDDSCAT Version Tag\t"
			"Frequency (GHz)\tDipole Spacing (um)\t"
			"M_real\tM_imag\tAeff (um)\tBetas\tThetas\tPhis\tNumber of Raw Orientations Available\t"
			//"V_Voronoi\tSA_Voronoi\tf_Voronoi\tV_Convex\tSA_Convex\tf_Convex\t"
			//"V_Ellipsoid_Max\tSA_Ellipsoid_Max\tEllipsoid_Max\t"
			//"V_Circum_Sphere\tSA_Circum_Sphere\tf_Circum_Sphere\t"
			"Qsca_iso\tQbk_iso\tQabs_iso\tQext_iso"<< endl;
		
		using namespace boost::filesystem;
		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				return pf;
			} else {
				return p;
			}
		};

		vector<path> inputs, shapes;
		for (const std::string &rin : vsInput)
		{
			path p(rin);
			path ps = expandSymlinks(p);
			inputs.push_back(ps);
		}
		for (const std::string &rin : vsShapes)
		{
			path p(rin);
			path ps = expandSymlinks(p);
			shapes.push_back(ps);
		}

		for (const path &p : inputs)
		{
			cerr << "Processing: " << p << endl;
			using namespace rtmath::ddscat;
			boost::shared_ptr<ddOutput> ddOut(new ddOutput);

			if (is_directory(p))
				ddOut = ddOutput::generate(p.string(), true);
			else if (Ryan_Serialization::known_format(p))
				ddOut->readFile(p.string());
			else if (p.extension() == ".avg")
			{
				// Attempt to match a shape to the avg file
				ddOut->avg = boost::shared_ptr<ddOutputSingle>(new ddOutputSingle(p.string(), "avg"));
				
				std::string aPrefix = rtmath::ddscat::dataset::getPrefix(p);
				cerr << "aPrefix " << aPrefix << endl;
				boost::filesystem::path ps;
				bool found = false;
				for (const auto &s : shapes)
				{
					std::string sPrefix = rtmath::ddscat::dataset::getPrefix(s);
					cerr << "\t" << s << "\t" << sPrefix << endl;
					if (aPrefix == sPrefix)
					{
						ps = s;
						break;
					}
				}
				if (!ps.string().size())
				{
					cerr << "Cannot match " << p << " to any provided shape file.\n";
					continue;
				}

				shapefile::shapefile shp(ps.string());
				ddOut->shapeHash = shp.hash();
			}
			else
			{
				cerr << "\tWrong / unknown file type for this program.\n";
				continue;
			}

			// out << "Filename\tDescription\tShape Hash\tDDSCAT Version Tag\t"
			// "Frequency (GHz)\tDipole Spacing (um)\t"
			// "M_real\tM_imag\tAeff (um)\tBetas\tThetas\tPhis\tNumber of Raw Orientations Available\tDipole Spacing\t"
			// "V_Voronoi\tSA_Voronoi\tf_Voronoi\tV_Convex\tSA_Convex\tf_Convex\t"
			// "V_Ellipsoid_Max\tSA_Ellipsoid_Max\tEllipsoid_Max\t"
			// "V_Circum_Sphere\tSA_Circum_Sphere\tf_Circum_Sphere\t"
			// "Qsca_iso\tQbk_iso\tQabs_iso\tQext_iso"<< endl;
			//ddOut->parfile->getRots(rots);

			// Recover from segfaulted runs when done in a script.
			if (!ddOut->avg) continue;
			rotations rots;
			ddOut->avg->getRots(rots);
			/*
			if (!ddOut->stats)
			{
				if (ddOut->shape)
					ddOut->stats = boost::shared_ptr<rtmath::ddscat::stats::shapeFileStats>(stats::shapeFileStats::genStats(ddOut->shape));
			}
			// Calculate scaled quantities
			double ds = ddOut->avg->dipoleSpacing();

			double Vvoro = ddOut->stats->SVoronoi_hull.V * pow(ds,3.);
			double Svoro = ddOut->stats->SVoronoi_hull.SA * pow(ds,2.);
			double fvoro = ddOut->stats->SVoronoi_hull.f;

			double Vconv = ddOut->stats->Sconvex_hull.V * pow(ds,3.);
			double Sconv = ddOut->stats->Sconvex_hull.SA * pow(ds,2.);
			double fconv = ddOut->stats->Sconvex_hull.f;

			double Vellm = ddOut->stats->Sellipsoid_max.V * pow(ds,3.);
			double Sellm = ddOut->stats->Sellipsoid_max.SA * pow(ds,2.);
			double fellm = ddOut->stats->Sellipsoid_max.f;

			double Vcirc = ddOut->stats->Scircum_sphere.V * pow(ds,3.);
			double Scirc = ddOut->stats->Scircum_sphere.SA * pow(ds,2.);
			double fcirc = ddOut->stats->Scircum_sphere.f;
			*/

			if (sDescrip.size()) ddOut->description = sDescrip;

			out << p.string() << "\t" << ddOut->description << "\t"
				<< ddOut->shapeHash.lower << "\t" << ddOut->ddvertag << "\t"
				<< ddOut->avg->freq() << "\t" << ddOut->avg->dipoleSpacing() << "\t"
				<< ddOut->avg->getM().real() << "\t" << ddOut->avg->getM().imag() << "\t"
				<< ddOut->avg->aeff() << "\t" << rots.bN() << "\t" << rots.tN() << "\t" << rots.pN() << "\t"
				<< ddOut->scas.size() << "\t" // << ds << "\t"
				//<< Vvoro << "\t" << Svoro << "\t" << fvoro << "\t" 
				//<< Vconv << "\t" << Sconv << "\t" << fconv << "\t" 
				//<< Vellm << "\t" << Sellm << "\t" << fellm << "\t" 
				//<< Vcirc << "\t" << Scirc << "\t" << fcirc << "\t" 
				<< ddOut->avg->getStatEntry(stat_entries::QSCAM) << "\t"
				<< ddOut->avg->getStatEntry(stat_entries::QBKM) << "\t"
				<< ddOut->avg->getStatEntry(stat_entries::QABSM) << "\t"
				<< ddOut->avg->getStatEntry(stat_entries::QEXTM) << "\n";

		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


