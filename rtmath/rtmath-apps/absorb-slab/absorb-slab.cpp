/* absorb-slab
* A program designed to go through the absorption functions and test their domains for a sample, 
* really crude bulk atmosphere. The block atmosphere is a 5 km-thick slab with 700 mb pressure.
* It is at 273 K. Rel. humid. is at 15%. It will show if any of the referenced absorption 
* functions is misbehaving. */

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <boost/tokenizer.hpp>
#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::atmos;

	try {
		if (argc == 1) doHelp();
		config::parseParams p(argc,argv);
		cerr << "rtmath-absorb-slab\n\n";
		bool flag = false;

		p.readParam("-h",flag);
		if (flag) doHelp();

		double rh = 0, T = 273, pres = 700, dz = 5;

		double f[3] = {0,0,0};
		{
			string strF;
			flag = p.readParam("-f", strF);
			if (!flag)
			{
				cerr << "Frequency range must be specified.\n";
				doHelp();
			}
			// Tokenize the string
			// The way that I wrote it, connand::processCommands
			// cannot yet do this level of parsing. Too bad.
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",:");
			tokenizer tokens(strF, sep);
			std::vector<double> fs;
			for (tokenizer::iterator tok_iter = tokens.begin();
				tok_iter != tokens.end(); ++tok_iter)
				fs.push_back( atof(tok_iter->c_str()) );
			if (fs.size() < 3)
			{
				f[0] = fs[0];
				f[2] = fs[0];
				f[1] = 1;
				cerr << "Frequency: " << f[0] << " GHz" << endl;
			} else {
				f[0] = fs[0];
				f[1] = fs[1];
				f[2] = fs[2];
				cerr << "Frequency range (GHz): from " << f[0] << 
					" to " << f[2] << " step " << f[1] << endl;
			}
			if (f[0] <= 0 || f[2] < f[0] || f[1] <= 0 || f[2] <= 0)
			{
				cerr << "Bad frequency input.\n";
				doHelp();
			}
		}

		set<string> gases;
		{
			string sGases;
			flag = p.readParam("-g", sGases);
			if (flag)
			{
				typedef boost::tokenizer<boost::char_separator<char> >
					tokenizer;
				boost::char_separator<char> sep(",");
				tokenizer tokens(sGases, sep);
				std::vector<double> fs;
				for (tokenizer::iterator it = tokens.begin();
					it != tokens.end(); ++it)
				{
					if (gases.count(*it) == 0)
						gases.insert(*it);
				}
			} else { // Using default gases instead
				gases.insert("H2O");
				gases.insert("O2");
				gases.insert("N2");
				gases.insert("COLLIDE");
			}
		}

		if (gases.count("H2O")) rh = 15; // Set new default value
		p.readParam("-rh", rh);
		p.readParam("-t", T);
		p.readParam("-p", pres);
		p.readParam("-dz", dz);

		// Build the sample atmosphere
		rtmath::atmos::atmos a;

		a._layers.resize(1);

		atmoslayer *layer = &a._layers[0];
		layer->dz(dz);
		layer->p(pres);
		layer->T(T);

		// Insert the gases into the layer
		{
			// Convert relative humidity into rho_Wat
			double rhoWat = 0;
			rhoWat = absorber::_Vden(layer->T(), rh);
			absorber *newgas;
			if (gases.count("H2O"))
			{

				newgas = new abs_H2O;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptrd(newgas); 
				layer->absorbers.insert(ptrd);
			}

			if (gases.count("O2"))
			{
				newgas = new abs_O2;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptrc(newgas); 
				layer->absorbers.insert(ptrc);
			}

			if (gases.count("N2"))
			{
				newgas = new abs_N2;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptrb(newgas); 
				layer->absorbers.insert(ptrb);
			}

			if (gases.count("COLLIDE"))
			{
				newgas = new collide;
				newgas->setLayer(*layer);
				newgas->wvden(rhoWat);
				std::shared_ptr<absorber> ptr(newgas); 
				layer->absorbers.insert(ptr);
			}
		}

		// Okay, now run the atmosphere for the frequency range requested and report results.
		{
			cerr << "Running atmosphere\n";
			cerr << "frequency (GHz), tau (nepers)" << endl;
			// Construct set of frequencies to analyze. Be nice and make sure
			// that the end frequency is in the set
			set<double> freqs;
			for (double fr = f[0]; fr <= f[2]; fr+=f[1])
				freqs.insert(fr);
			if (freqs.count(f[2]) == 0) freqs.insert(f[2]);

			// Loop through freqs
			set<double>::const_iterator it;
			for (it = freqs.begin(); it != freqs.end(); it++)
			{
				double tau = a.tau(*it);
				cout << *it << "," << tau << endl;
			}
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
#ifdef _WIN32
		std::getchar();
#endif
		return 1;
	}
	return 0;
}


void doHelp()
{
	using namespace std;
	cout << "rtmath-absorb-slab\n";
	cout << "A program for calculating transmittance in a sample atmosphere,\n";
	cout << "to determine the domains of the absorbtion-calculationg functions.\n";
	cout << "Options:\n";
	cout << "-f (frequency range)\n";
	cout << "\tSpecify the range of frequencies (in GHz) for\n";
	cout << "\ttransmittance calculation. Either specify a\n";
	cout << "\tsingle frequency, or specify a set of frequencies\n";
	cout << "\tusing the form (start,increment,stop)." << endl;
	cout << "-g (gases)\n";
	cout << "\tManually specify the gases to place in the atmosphere.\n";
	cout << "\tAcceptable values: H2O, N2, O2, COLLIDE\n";
	cout << "-rh (relative humidity as a percent)\n";
	cout << "\tSpecify relative humidity from 0 to 100.\n";
	cout << "\tWorks even without H2O, as some gas calculations \n";
	cout << "\t(like O2) depend on this value.\n";
	cout << "\tDefault is 0 w/o water, 15 w/water.\n";
	cout << "-t (temp, K)\n";
	cout << "\tTemperature (default = 273 K)\n";
	cout << "-p (pressure, hPa)\n";
	cout << "\tPressure (default = 700 hPa)\n";
	cout << "-dz (depth, km)\n";
	cout << "\tSlab thickness (defult 5 km)\n";
	cout << "-h\n";
	cout << "\tProduce this help message.\n";
	cout << endl << endl;
#ifdef _WIN32
	std::getchar();
#endif
	exit(1);
}

