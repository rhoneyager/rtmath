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
                // set< boost::tuple< paramSet<double>, MANIPULATED_QUANTITY::MANIPULATED_QUANTITY, string > > sizes;
                gen.sizes.insert(
                    boost::tuple< paramSet<double>, MANIPULATED_QUANTITY::MANIPULATED_QUANTITY, string >(
                        paramSet<double>("50:10:350"),
                        rtmath::ddscat::MANIPULATED_QUANTITY::REFF,
                        "um"
                        )
                    );
                
                // Need rotations   set<rotations> rots;
                rtmath::ddscat::rotations rots;
                gen.rots.insert(rots);
                
                // Need scattering angles
                
                
                MARK();
                gen.generate(gen.outLocation);
                //gen.write(gen.outLocation);
                
                std::ofstream outt("out.txt");
                boost::archive::text_oarchive ot(outt);
                ot << gen;
                
                ddscat::ddParGenerator::write(gen,"out.xml");
                
                //std::ofstream outx("out.xml");
                //boost::archive::xml_oarchive oa(outx);
                //oa << BOOST_SERIALIZATION_NVP(gen);

                
                // Set frequencies
/*
		// Set frequencies
		gen.freqs.insert(hasUnits(94.0, "GHz"));
		gen.freqs.insert(hasUnits(110.0, "GHz"));

		// Set temperature
		gen.temps.insert(hasUnits(-10.0, "C"));
		gen.temps.insert(hasUnits(267.0, "K"));

		// Set the shape rotations
		rotations ra; // use default values
		rotations rb(0,0,1,0,90,18,0,0,1);
		gen.rots.insert(ra);
		gen.rots.insert(rb);

		std::shared_ptr<shapeEllipsoid> bshape(new shapeEllipsoid());
		gen.setShapeBase(bshape);

		// The important constraints are now set. The output-writing routine
		// will write out the description file, and the generating routine will
		// generate the full directory structure.

		// Generate
		gen.generate("testout");

		// Write the output
		gen.write("test.txt");
                */
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		return 1;
	}
	return 0;
}

