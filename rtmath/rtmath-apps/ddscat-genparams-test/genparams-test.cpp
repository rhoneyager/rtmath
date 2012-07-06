#include <memory>
#include <iostream>
#include "../../rtmath/rtmath/rtmath.h"

int main(int argc, char **argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::debug;
	using namespace rtmath::ddscat;
	using namespace rtmath::units;
	try {
		cerr << "Batch genparams test program" << endl;
		rtmath::debug::appEntry(argc, argv);
		rtmath::config::parseParams p(argc, argv);

		// Will create a test set of ddscat runs
		// these will vary frequencies, sizes and shapes for a set of test cases
		// The purpose is to ascertain, develop and debug the corresponding library routines.
		// Also, the routines will be used in qt and ncurses programs

		// Use the default generator
		ddscat::ddParGenerator gen;
/*
		// Set name
		gen.name = "Test runs";

		// Set description
		gen.description = "genparams-test test to ensure proper file generation";
		gen.outLocation = "./out/";
		gen.baseParFile = "ddscat.par";
		gen.shapeStats = true;
		gen.temps.insert(std::make_pair<paramSet<double>, std::string>
			( paramSet<double>("263:5:270"), "K" )   );
		gen.temps.insert(std::make_pair<paramSet<double>, std::string>
			( paramSet<double>(230.0), "K" )   );

		std::map<std::string, std::string> freqaliases;
		freqaliases["GPM_Radiometer"] = "10.7, 18.7, 23.8, 89.0, 165.5, 183.3";
		freqaliases["GPM_Dual_Radar"] = "13.6, 35.5";
		gen.freqs.insert(std::make_pair<paramSet<double>, std::string>
			( paramSet<double>("35.5,GPM_Radiometer,GPM_Dual_Radar", &freqaliases), "GHz" )   );

		// Need sizes   
		gen.shapeConstraintsGlobal.insert( // Effective radius
			std::make_shared<shapeConstraint>("reff", "50:10:350", "um")
			);
		gen.shapeConstraintsGlobal.insert( // Interdipole spacing
			std::make_shared<shapeConstraint>("d", "5", "um")
			);

		// Set the shapes (two ellipsoids, one of which is really a sphere)
		auto shp = std::make_shared<rtmath::ddscat::shapes::ellipsoid>();
		gen.shapes.insert(shp);
		shp->shapeConstraints.insert(std::make_shared<shapeConstraint>("shpar1","1"));
		shp->shapeConstraints.insert(std::make_shared<shapeConstraint>("shpar2","1"));
		shp->shapeConstraints.insert(std::make_shared<shapeConstraint>("shpar3","1"));

		shp = std::make_shared<rtmath::ddscat::shapes::ellipsoid>();
		gen.shapes.insert(shp);
		shp->shapeConstraints.insert(std::make_shared<shapeConstraint>("shpar1","1"));
		shp->shapeConstraints.insert(std::make_shared<shapeConstraint>("shpar2","2"));
		shp->shapeConstraints.insert(std::make_shared<shapeConstraint>("shpar3","3"));

		// Need rotations   set<rotations> rots;
		rtmath::ddscat::rotations rots;
		gen.rots.insert(rots);

		// Scattering angles should have already been provided by ddParGenerator::import




		// Attempt to generate everything
		gen.generate(gen.outLocation);
		ddscat::ddParGenerator::write(gen,gen.outLocation);

		// Verify read
		ddscat::ddParGenerator ver;
		ddscat::ddParGenerator::read(ver,gen.outLocation);
*/		
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		return 1;
	}
	catch (std::exception &e)
	{
		cerr << "exception caught: " << e.what() << endl;
		return 2;
	}
	return 0;
}

