/* conv-units is a program that tests and implements unit conversions with the rtmath library.
   This is more powerful than just standard unit conversions, as conversions relying on
   equations are also supported (for example, interconversions between frequency and wavelength)
   */

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <map>
#include <vector>
#include <boost/lexical_cast.hpp>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	try {
		// Process some of the flags
		rtmath::debug::appEntry(argc, argv);
		string inUnits, outUnits;
		double in, out;
		{
			//if (argc == 1) doHelp();
			config::parseParams p(argc, argv);
			bool flag;
			if (p.readParam("-h")) doHelp();

			vector<string> inProc;


			flag = p.readParam<string>("-i", inProc);
			if (!flag)
			{
				cout << "Input quantity (no units): ";
				cin >> in;
				cout << "Input units: ";
				cin >> inUnits;
			} else {
				if (inProc.size() != 2) doHelp();
				inUnits = inProc[1];
				in = boost::lexical_cast<double>(inProc[0]);
			}

			flag = p.readParam<string>("-o", outUnits);
			if (!flag)
			{
				cout << "Output units: ";
				cin >> outUnits;
			}
		}
		
		// Do only spectral conversions for now
		units::conv_spec cnv(inUnits,outUnits);
		out = cnv.convert(in);
		cout << out << endl;
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	}
	return 0;
}

void doHelp()
{
	using namespace std;
	cerr << "rtmath-conv-units\n";
	cerr << "Interconvert between different units with equation support.\n";
	cerr << "Options:\n";
	cerr << "-i (quantity) (input units)\n";
	cerr << "-o (output units)\n";
	exit(1);
}


