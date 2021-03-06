#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <memory>
//#include <netcdf.h>
#include <cmath>
#include "../rtmath/ddscat/ddscat.h"
#include "../rtmath/ddscat/ddweights.h"
#include "../rtmath/units.h"

namespace rtmath {

	namespace ddscat {
		/*
		void ddOutputSingle::size(std::set<double> &thetas, std::set<double> &phis) const
		{
			thetas.clear();
			phis.clear();

			std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it;
			for (it=_fs.begin(); it != _fs.end(); it++)
			{
				double th = it->first.theta;
				if (thetas.count(th) == 0)
					thetas.insert(th);
				double ph = it->first.phi;
				if (phis.count(ph) == 0)
					phis.insert(ph);
			}
		}

		void ddOutputSingle::interpolate(const ddCoords &coords, ddScattMatrix &res) const
		{
			// I already have the set of scattering matrices available. I'll use them
			// to perform a linear interpolation. This interpolation may be crude, but
			// it is necessary it I want to generate matrices with angles along the appropriate
			// quadrature points. I really with that ddscat could do this for me.

			// Conveniently, all of my ddscat stuff varies theta from 0 to 180, while
			// phi is either 0 or 90 degrees (covering co- and orthogonal polarizations).

			// So, I really should look at phi first
			// Find the four matrices needed for the planar average
			throw rtmath::debug::xUnimplementedFunction();
			// Store the resulting angles here
			ddCoords col, cou, cxl, cxu;

			std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator it;
			for (it = _fs.begin(); it != _fs.end(); ++it)
			{
				// Find best value underneath and overshooting for each phi
				if (it->first.theta > col.theta && it->first.phi == 0 && it->first.theta <= coords.theta)
					col = it->first;
				if (it->first.theta > cxl.theta && it->first.phi == 90 && it->first.theta <= coords.theta)
					cxl = it->first;
				if (it->first.theta < cou.theta && it->first.phi == 0 && it->first.theta >= coords.theta)
					cou = it->first;
				if (it->first.theta > cxu.theta && it->first.phi == 90 && it->first.theta >= coords.theta)
					cxu = it->first;
			}

			// Okay, we have the set of the four closest coordinates on the sphere.
			// Average the scattmatrices for each phi
			double facttheta = (cou.theta - coords.theta) / (cou.theta - col.theta);
			double factphi = (cxu.phi - coords.phi) / (cxu.phi - cou.phi);

			res.lock = true;
			for (size_t i=0;i<4;i++)
			{
				for (size_t j=0;j<4;j++)
				{
					// Do interpolation on Pnn and Knn
					res.Pnn[i][j] = factphi * (facttheta * _fs[col].Pnn[i][j] + (1.0-facttheta) * _fs[cou].Pnn[i][j]);
					res.Pnn[i][j] += (1.0 - factphi) * (facttheta * _fs[cxl].Pnn[i][j] + (1.0-facttheta) * _fs[cxu].Pnn[i][j]);
					res.Knn[i][j] = factphi * (facttheta * _fs[col].Knn[i][j] + (1.0-facttheta) * _fs[cou].Knn[i][j]);
					res.Knn[i][j] += (1.0 - factphi) * (facttheta * _fs[cxl].Knn[i][j] + (1.0-facttheta) * _fs[cxu].Knn[i][j]);
				}
			}
		}

		void ddOutputSingle::emissionVector(double mu, matrixop &res) const
		{
			throw debug::xUnimplementedFunction();
			// Well, this is a function that is needed to generate the Stokes emission vector,
			// as required by the doubling-adding method for thermal radiation sources,
			// as are valid at microwave wavelengths

			// Both conveniently and annoyingly, many of the emission vector calculations 
			// are done at very odd values of mu. This is a result of the type of quadrature
			// used in the calculation. I don't have these quadrature points. Instead, I have 
			// 18+ possible angle combinations with which I can perform integration using one 
			// of the more standard Riemann sums.

			// The emission vector is defined as \sigma_l = K_l - \int P_l d\Omega'
			// That is, I need the extinction matrix (easy) and the integration of the 
			// Mueller scattering matrix over the surface area of a sphere. It would be great
			// to use spherical harmonics, but not easily paractical due to my selection of 
			// points.

			// Conveniently, assuming symmetry, I can convert the surface integral into that of
			// a more standard form.

			// If mu is the outgoing angle, and I'm integrating over all of the incoming 
			// angles, then I'm really very confused.
		}

		void ddOutputEnsembleGaussian::_genWeights(const std::set<double> &points,
			const std::map<double, unsigned int> &recs)
		{
			gaussianPosWeights a(_sigma, points);
			a.getWeights(_weights);

			// Tweak weights to account for multiple phi recurrances
			// Tweak divided the weight by the number of times it pops up

			std::set<double>::const_iterator it;
			double sum = 0;
			for (it = points.begin(); it != points.end(); it++)
			{
				_weights[*it] /= (double) recs.at(*it);
				sum += _weights[*it];
			}

			// And then they all get rescaled to unity
			for (it = points.begin(); it != points.end(); it++)
			{
				_weights[*it] /= sum;
			}

		}

		void ddOutputEnsembleGaussianPhi::generate()
		{
			// ensemble should be set. 
			// First, collect all phi into a set and generate the weights
			using namespace std;
			set<double> phis;
			map<double, unsigned int> phiRecs;
			std::map<ddCoords3, ddOutputSingle, ddCoordsComp>::const_iterator it, ot, kt;
			for (it = ensemble.begin(); it != ensemble.end(); it++)
			{
				// Note: normally, it would fail if multiple phis are introduced.
				// I'm just recording the recurrance for a weight adjustment later on.
				// Either that, or I could sum over those elements with the same phi,
				// but that will lead to an extra summation step later on
				double val = it->first.phi;
				if (phis.count(val) == 0)
					phis.insert(val);
				if (phiRecs.count(val))
					phiRecs[val]++;
				else phiRecs[val] = 1;
			}

			_genWeights(phis, phiRecs);

			// We now have weights. Time to take the elements in the ensemble
			// and sum according to the weighting function results.

			res._fs.clear();

			matrixop Peff(2,4,4), Keff(2,4,4), Pn(2,4,4), Kn(2,4,4);
			for (it = ensemble.begin(); it != ensemble.end(); it++)
			{
				// Iterate through and match the _fs element with res' element

				std::map<ddCoords, ddScattMatrix, ddCoordsComp>::iterator resf;
				std::map<ddCoords, ddScattMatrix, ddCoordsComp>::const_iterator srcf, srcfp;
				for (srcf=it->second._fs.begin(); srcf!=it->second._fs.end(); srcf++)
				{
					// Find the matching resf based on the key
					resf = res._fs.find(srcf->first);
					if (resf == res._fs.end()) // Iterator not found. Add a new key.
					{
						std::pair< std::map<ddCoords, ddScattMatrix, ddCoordsComp>::iterator, bool> op;
						ddCoords a( srcf->first );
						ddScattMatrix b(a.theta, a.phi, 0);
						// TODO: check how fs add together. WOuld be easier that way.
						b.lock = true; // Suppress overwriting of P and K (normally calculated from vals[][])
						res._fs[a] = b;
						resf = res._fs.find(srcf->first); // Ugly, but fast to code
					}
					// resf is the iterator pointing to the ensemble result's value for _fs
					// So, it points to a ddScattMartix
					// srcf is the iterator pointing to the current scattMatrix which is 
					// multiplied by the weight and then added to the value in resf.

					// Using matrixops for ease (and not having to write yet another set of loops)

					double wt = 0;
					wt = weight(it->first);

					matrixop Peff(2,4,4), Keff(2,4,4);
					matrixop Pn(2,4,4),   Kn(2,4,4);
					Peff.fromDoubleArray(&(resf->second.Pnn)[0][0]);
					Keff.fromDoubleArray(&(resf->second.Knn)[0][0]);
					Pn.fromDoubleArray(&(srcf->second.Pnn)[0][0]);
					Kn.fromDoubleArray(&(srcf->second.Knn)[0][0]);

					Peff = Peff + (Pn * wt);
					Keff = Keff + (Kn * wt);

					Peff.toDoubleArray(&(resf->second.Pnn)[0][0]);
					Keff.toDoubleArray(&(resf->second.Knn)[0][0]);
				}
			}
		}

		double ddOutputEnsembleGaussianPhi::weight(const ddCoords3 &coords) const
		{
			if (_weights.count(coords.phi))
				return _weights.at(coords.phi);
			throw rtmath::debug::xAssert("For some reason, the appropriate weight was not computed!");
			return 0;
		}
		*/
	}; // end namespace ddscat

}; // end namespace rtmath

