/* This program produces shapefiles based on rectangular prisms.
 * Multiple prisms may be selected, and 'negative' regions may 
 * be excised. This allows for the generation of a few rather 
 * useful test cases.
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
#include <boost/shared_array.hpp>

#include "../../rtmath/rtmath/common_templates.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/denseMatrix.h"


int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-shape-rectprism\n\n";
		rtmath::debug::appEntry(argc, argv);

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("fill", -1);

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("output,o", po::value<string>(), "This is the output filename. If unspecified, write to stdout.")
			("fill,f", po::value< vector<string> >(), "Specify a region to be filled. Region selected as a colon and comma-separated "
			"list of sx:ex,sy:ey,sz:ez[,sign] where s is the start and e is the end vertex. Sign is optional. If "
			"positive or unspecified, add the selected point. If negative, remove the elected point.");
		// TODO: allow for shape distortions

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);    

		if (vm.count("help") || argc == 1) {
			cerr << desc << "\n";
			return 1;
		}

		ostream *pout = nullptr;
		ofstream ofile;
		if (vm.count("output"))
		{
			ofile.open(vm["output"].as<string>().c_str());
			pout = &ofile;
		} else {
			pout = &cout;
		}
		ostream &out = *pout;

		vector<string> fills = vm["fill"].as< vector<string> >();

		// A lambda determining how the point ranges are split
		auto fSplit = [](string inval, int mins[3], int maxs[3], int &sign)
		{
			sign = 1;
			typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
			boost::char_separator<char> sepcom(",");
			boost::char_separator<char> sepcol(":");
			tokenizer tp(inval, sepcom);
			size_t selector = 0;
			for (auto it = tp.begin(); it != tp.end(); ++it, ++selector)
			{
				if (selector < 3)
				{
					tokenizer tc(*it, sepcol);
					auto p = tc.begin();
					if (p == tc.end()) throw;
					mins[selector] = boost::lexical_cast<int>(*p);
					++p;
					if (p == tc.end())
						maxs[selector] = mins[selector];
					else
						maxs[selector] = boost::lexical_cast<int>(*p);
				} else {
					sign = boost::lexical_cast<int>(*it);
					return;
				}
			}
		};

		// Do two passes. First pass allows for dimensioning and memory allocation.
		// Second pass fills in the shape into memory. Dense memory layout is selected, since we are dealing with rectangles.
		using namespace boost::accumulators;
		accumulator_set<int, stats<tag::min, tag::max> > sx, sy, sz;
		accumulator_set<int, stats<tag::sum> > svolmax;
		cerr << "Pass 1" << endl;
		for (auto it = fills.begin(); it != fills.end(); ++it)
		{
			int imins[3], imaxs[3], sign;
			fSplit(*it, imins, imaxs, sign);
			sx(imins[0]);
			sx(imaxs[0]);
			sy(imins[1]);
			sy(imaxs[1]);
			sz(imins[2]);
			sz(imaxs[2]);
			svolmax( abs( (imaxs[0]-imins[0]+1) * (imaxs[1]-imins[1]+1) * (imaxs[2]-imins[2]+1) ) );
		}


		// svolmax now has the max amount of memory required for processing.
		size_t sizeX = boost::accumulators::max(sx) - boost::accumulators::min(sx) + 1;
		size_t sizeY = boost::accumulators::max(sy) - boost::accumulators::min(sy) + 1;
		size_t sizeZ = boost::accumulators::max(sz) - boost::accumulators::min(sz) + 1;
		denseMatrix dm(sizeX, sizeY, sizeZ);
		size_t offsetX = boost::accumulators::min(sx);
		size_t offsetY = boost::accumulators::min(sx);
		size_t offsetZ = boost::accumulators::min(sx);
		
		cerr << "Max possible dimensions of final shape:" << endl;
		cerr << "x - " << boost::accumulators::min(sx) << ":" << boost::accumulators::max(sx) << endl;
		cerr << "y - " << boost::accumulators::min(sy) << ":" << boost::accumulators::max(sy) << endl;
		cerr << "z - " << boost::accumulators::min(sz) << ":" << boost::accumulators::max(sz) << endl;
		cerr << endl;

		// Iterate over each fill statement. Expand fill statement. Everything exists within a PCL cloud for now.
		cerr << "Pass 2" << endl;
		for (auto it = fills.begin(); it != fills.end(); ++it)
		{
			int imins[3], imaxs[3], sign;
			fSplit(*it, imins, imaxs, sign);
			bool bsign = (sign>0) ? true : false;

			// Iterate over all possible x, y, z and set the matrix accordingly
			for (int x = imins[0]; x <= imaxs[0]; x++)
			{
				for (int y = imins[1]; y <= imaxs[1]; y++)
				{
					for (int z = imins[2]; z <= imaxs[2]; z++)
					{
						size_t ix = (size_t) (x) - offsetX;
						size_t iy = (size_t) (y) - offsetY;
						size_t iz = (size_t) (z) - offsetZ;
						dm.set(ix, iy, iz, bsign);
					}
				}
			}
		}

		// A final pass is needed to calculate the shape center of mass
		cerr << "Pass 3" << endl;
		accumulator_set<double, stats<tag::mean> > mX, mY, mZ;
		for (int x = boost::accumulators::min(sx); x <= boost::accumulators::max(sx); x++)
		{
			for (int y = boost::accumulators::min(sy); y <= boost::accumulators::max(sy); y++)
			{
				for (int z = boost::accumulators::min(sz); z <= boost::accumulators::max(sz); z++)
				{
					mX(x - offsetX);
					mY(y - offsetY);
					mZ(z - offsetZ);
				}
			}
		}
		// Do direct writeout of shape. No need to invoke shapefile, as this would produce another copy operation.

		cerr << "Writing file" << endl;
		out << "rtmath-shape-rectprism output\n";
		out << "\t" << dm.numOn << "\t= Number of lattice points" << endl;
		out << "\t1\t0\t0\t= target vector a1 (in TF)\n";
		out << "\t0\t1\t0\t= target vector a2 (in TF)\n";
		out << "\t1\t1\t1\t= d_x/d  d_y/d  d_x/d  (normally 1 1 1)\n";
		out << "\t" << boost::accumulators::mean(mX) + offsetX
			<< "\t" << boost::accumulators::mean(mY) + offsetY
			<< "\t" << boost::accumulators::mean(mZ) + offsetZ
			<< "\t= X0(1-3) = location in lattice of target origin" << endl;
		out << "\tNo.\tix\tiy\tiz\tICOMP(x, y, z)" << endl;

		size_t i=1;
		for (size_t x=0; x<sizeX; x++)
		{
			for (size_t y=0; y<sizeY; y++)
			{
				for (size_t z=0; z<sizeZ; z++)
				{
					if (dm.get(x,y,z))
					{
						out << "\t" << i
							<< "\t" << x + offsetX 
							<< "\t" << y + offsetX
							<< "\t" << z + offsetZ
							<< "\t1\t1\t1\n";
						i++;
					}
				}
			}
		}
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

