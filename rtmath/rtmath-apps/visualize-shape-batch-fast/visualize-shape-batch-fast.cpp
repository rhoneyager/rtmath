// Program produces silo visualizations of shapes + voronoi data and generates a VisIt script to save relevant images
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/macros.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;
	try
	{
		using namespace std;
		cerr << "rtmath-visualize-shape-batch-fast\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("inputshp,s", po::value< vector<string> >()->multitoken(), "Input shape files")
			("output,o", po::value<string>()->default_value("ingest.py"), "specify output VisIt script")
			
			("force-hash-dir", po::value<string>(), "force hash directory")
			("basic-plots", po::value<bool>()->default_value(true), "Plot basic contoured shape at four orientations")
			;
		po::positional_options_description p;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		vector<string> inputshp;
		if (vm.count("inputshp"))
		{
			inputshp = vm["inputshp"].as< vector<string> >();
			cerr << "Input shape files are:" << endl;
			for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
				cerr << "\t" << *it << "\n";
		};

		string output;

		if (vm.count("output"))
		{
			output = vm["output"].as<string>();
			cerr << "Outputting to: " << output << endl;
		}

		bool force_hash_dir = false;
		string hdir;
		if (vm.count("force-hash-dir")) {
			force_hash_dir = true;
			hdir = vm["force-hash-dir"].as<string>();
		}

		bool basicPlots = vm["basic-plots"].as<bool>();


		ofstream ofile(output.c_str());

		ofile << "# Automatically-generated ingest script to produce visualizations\n";
		ofile << endl << endl;


		// Validate input files
		vector<string> vinputs;
		for (auto it = inputshp.begin(); it != inputshp.end(); ++it)
		{
			path pi(*it);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(it->c_str());
			if (is_directory(pi))
			{
				path ps = pi / "shape.dat";
				if (exists(ps)) vinputs.push_back(ps.string());
				//else continue;
				else {
					// Iterate over directory (one level) and pull all hdf5 files
					using namespace boost::filesystem;
					vector<path> cands;
					copy(directory_iterator(pi),
						directory_iterator(), back_inserter(cands));
					for (const auto &f : cands)
					{
						if (is_directory(f)) continue;
						std::string unc, meth;
						Ryan_Serialization::uncompressed_name(f.string(), unc, meth);
						if (path(unc).extension().string() == ".hdf5")
							vinputs.push_back(f.string());
					}
				}
				//else throw rtmath::debug::xPathExistsWrongType(it->c_str());
			}
			else vinputs.push_back(*it);
		}

		

		
		using std::vector;
		using namespace rtmath::ddscat;
		
		for (auto it = vinputs.begin(); it != vinputs.end(); ++it)
		{
			vector<boost::shared_ptr<shapefile::shapefile> > shapes;
			cerr << "Processing " << *it << endl;
			auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
			iopts->filename(*it);
			try {
				// Handle not needed as the read context is used only once.
				if (shapefile::shapefile::canReadMulti(nullptr, iopts))
					shapefile::shapefile::readVector(nullptr, iopts, shapes);
				else {
					boost::shared_ptr<shapefile::shapefile> s(new shapefile::shapefile);
					s->readFile(*it);
					shapes.push_back(s);
				}

				for (const auto &vd : shapes)
				{
					cerr << "Processing hash " << vd->hash().lower << endl;

					string filename("silos/");
					filename.append(vd->hash().string());
					filename.append(".silo");

					auto opts = registry::IO_options::generate();
					opts->filename(filename);

					std::shared_ptr<registry::IOhandler> handle;
					handle = vd->writeMulti(handle, opts);
					//handle = vd.second->writeMulti(handle, opts);

					// And write the script portion
					// Contour basic data
					ofile << "OpenDatabase(\"localhost:" << filename << "\", 0)\n"
						"DeleteAllPlots()\n"
						"AddPlot(\"Contour\", \"Shp_Mesh_Dielectrics\", 1, 1)\n"
						"ContourAtts = ContourAttributes()\n"
						"ContourAtts.colorType = ContourAtts.ColorBySingleColor\n"
						"ContourAtts.legendFlag = 0\n"
						"ContourAtts.singleColor = (255, 0, 0, 255)\n"
						"ContourAtts.contourNLevels = 1\n"
						"ContourAtts.contourMethod = ContourAtts.Level\n"
						"ContourAtts.scaling = ContourAtts.Linear\n"
						"ContourAtts.wireframe = 0\n"
						"SetPlotOptions(ContourAtts)\n"
						"DrawPlots()\n\n";

					// Take pictures from different angles
					auto pic = [&](float nx, float ny, float nz, float ux, float uy, float uz, const std::string &suffix)
					{
						ofile << "ResetView()\n"
							"View3DAtts = View3DAttributes()\n"
							"View3DAtts.viewNormal = (" << nx << ", " << ny << ", " << nz << ")\n"
							"View3DAtts.viewUp = (" << ux << ", " << uy << ", " << uz << ")\n"
							"View3DAtts.viewAngle = 30\n"
							"View3DAtts.parallelScale = 35.6861\n"
							"View3DAtts.nearPlane = -71.3723\n"
							"View3DAtts.farPlane = 71.3723\n"
							"View3DAtts.imagePan = (0, 0)\n"
							"View3DAtts.imageZoom = 1\n"
							"View3DAtts.perspective = 1\n"
							"View3DAtts.eyeAngle = 2\n"
							"View3DAtts.centerOfRotationSet = 0\n"
							"View3DAtts.axis3DScaleFlag = 0\n"
							"View3DAtts.axis3DScales = (1, 1, 1)\n"
							"View3DAtts.shear = (0, 0, 1)\n"
							"SetView3D(View3DAtts)\n"
							"RecenterView()\n"
							;

						ofile << "SaveWindowAtts = SaveWindowAttributes()\n"
							"SaveWindowAtts.outputToCurrentDirectory = 1\n"
							"SaveWindowAtts.fileName = \"" << vd->hash().string() << suffix << "\"\n"
							"SaveWindowAtts.family = 0\n"
							"SaveWindowAtts.format = SaveWindowAtts.PNG\n"
							"SaveWindowAtts.width = 1440\n"
							"SaveWindowAtts.height = 1080\n"
							"SetSaveWindowAttributes(SaveWindowAtts)\n"
							"SaveWindow()\n\n";
					};

					pic(1, 0, 0, 0, 1, 0, "-1");
					pic(0, 1, 0, 0, 0, -1, "-2");
					pic(0, 0, 1, 0, 1, 0, "-3");
					pic(0.651427f, 0.531756f, 0.541182f, -0.240183f, 0.82114f, -0.517727f, "-4");

					// Close the database
					ofile << "OpenDatabase(\"localhost:" << filename << "\")" << std::endl;
				}
			}
			catch (std::exception &e)
			{
				cerr << e.what() << std::endl;
				continue;
			}
		}


		

		ofile << "\nexit()" << std::endl;

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


