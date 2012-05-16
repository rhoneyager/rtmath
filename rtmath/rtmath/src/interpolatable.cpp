#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/interpolatable.h"

namespace rtmath {
	namespace interpolation
	{

		ddOutputSingleInterp::ddOutputSingleInterp()
		{
			_init();
		}

		ddOutputSingleInterp::~ddOutputSingleInterp()
		{
		}

		void ddOutputSingleInterp::_init()
		{
			_clear();
			_interpMethod = BILINEAR;
		}

		void ddOutputSingleInterp::_clear()
		{
			_scattMatricesAll.clear();
			_scattMatricesRaw.clear();
			_interpMap.clear();
		}

		size_t ddOutputSingleInterp::_sizeRaw() const
		{
			return _scattMatricesRaw.size();
		}

		size_t ddOutputSingleInterp::_sizeRawNan() const
		{
			size_t nN = 0;
			// Iterate through scattering matrices. nans have property that a != a (IEEE standard).
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				double Pnn[4][4];
				(*it)->mueller(Pnn);
				for (size_t i=0;i<16;i++)
					if (&Pnn[i] != &Pnn[i])
					{
						nN++;
						break;
					}
			}
			return nN;
		}

		void ddOutputSingleInterp::_insert(std::shared_ptr<const ddscat::ddScattMatrix> &obj)
		{
			_scattMatricesAll.insert(obj);
			_scattMatricesRaw.insert(obj);
			// _interpMap addition is annoying, but I can use the old _interpMap[...] = ...
			// but I don't want to. The other, easier way is commented out immediately below.
			typedef std::unordered_map<coords::cyclic<double>, 
				std::shared_ptr<const ddscat::ddScattMatrix>, 
				boost::hash<coords::cyclic<double> > > mymap;
			_interpMap.insert(mymap::value_type(obj->genCoords(),obj));
			//_interpMap[obj->genCoords()] = obj;
		}

		void ddOutputSingleInterp::interpolate(
				const coords::cyclic<double> &pt,
				std::shared_ptr<const ddscat::ddScattMatrix> &res
				) const
		{
			using namespace std;
			res = nullptr;
			if (_interpMethod != BILINEAR) throw rtmath::debug::xUnimplementedFunction();
			throw rtmath::debug::xUnimplementedFunction();

			// Cacheing on an unordered map is certainly slow. But, is there a better way?
			if (_interpMap.count(pt))
			{
				res = _interpMap.at(pt);
				return;
			}

			// Unfortunately, matrixop only supports doubles at this time...
			// To handle the situation of real and complex components, 
			// create a matrixop that is doubled in the horizontal dimension to
			// handle the real and imaginary components separately.
			// Should work for bilinear interpolation.

			matrixop a(2,2,4), b(2,2,4); // really two matrixops

			if (_interpMethod == BILINEAR)
			{
				// Find the nearest points in the phase space. There are four nearest points,
				// typically. Distance is calculated in Eulerian flat coordinates.
				// These four points, by definition, are the closest
				// Since this is interpolation, not extrapolation, we may assume that the interpolation
				// is between the extrapolations
				// The four points are also located on a semi-regular grid
				// However, the phase difference in phi (0 and 90) screws with the distance calculation
				// somewhat, so the points might not be a square. They might be linearly-spaced!

				// To compensate for this, split pt into theta = 0 and 90 degree components.
				multimap<double, coords::cyclic<double> > nearestA, nearestB;
				if (_scattMatricesRaw.size() < 4) 
					throw rtmath::debug::xBadInput("Not enough points for bilinear interpolation.");
				for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
				{
					// Calculate distance
					// Also check that the frequencies match
					coords::cyclic<double> crds = (*it)->genCoords();
					coords::cyclic<double> pta(3,pt.get(0),pt.get(1),0);
					coords::cyclic<double> ptb(3,pt.get(0),pt.get(1),90);
					// Compare frequencies. Interpolation must occur with the same frequency.
					if (crds.get(0) != pt.get(0)) continue;
					double dista = pta.distance(crds);
					double distb = ptb.distance(crds);
					if (dista < 0) continue; // Skip entry in case of error.
					// Unfortunately, high-efficiency insertion is not possible...
					nearestA.insert( pair<double,coords::cyclic<double> >(dista,crds) );
					nearestB.insert( pair<double,coords::cyclic<double> >(distb,crds) );
				}

				// Check that enough nearest points have been found
				// (frequency filtering may have prevented this)
				if (nearestA.size() < 4)
					throw rtmath::debug::xBadInput("Not enough points for bilinear interpolation after frequency filtering.");

				// Take the four highest ranking points and use them to perform the interpolation
				// Not sure how to do this otherwise
				//     THESE ARE ALL dist-coords PAIRS
				auto ra = nearestA.begin();
				auto rb = ra;
				rb++;
				auto rc = nearestB.begin();
				rc++;
				auto rd=rc;
				rd++;

				double f, x, x1, x2, y, y1, y2;
				f = pt.get(0);
				x = pt.get(1);
				y = pt.get(2);
				x1 = ra->second.get(1);
				x2 = rd->second.get(1);
				y1 = ra->second.get(2);
				y2 = rd->second.get(2);

				// These should be spaced in a rectangular grid. If not, then interpolation will be a failure.
				// Verify the grid
				if (ra->second.get(2) == rc->second.get(2)) 
					throw rtmath::debug::xArrayOutOfBounds();
				if (rb->second.get(1) == rd->second.get(1)) 
					throw rtmath::debug::xArrayOutOfBounds();
				// Grid verified. It's at least quadrilateral, and should be rectangluar based on how ddscat works.

				// Bilinear interpolation formulae
				// f(P) = f(Q11)*(x2-x)(y2-y)/(x2-x1)(y2-y1)
				//      + f(Q21)*(x-x1)(y2-y)/(x2-x1)(y2-y1)
				//      + f(Q12)*(x2-x)(y-y1)/(x2-x1)(y2-y1)
				//      + f(Q22)*(x-x1)(y-y1)/(x2-x1)(y2-y1)
				double denom = (x2-x1)*(y2-y1);

				// Convert points into matrixops. Use ddScattMatrix comversion member.
				matrixop fa(2,2,4), fb(2,2,4), fc(2,2,4), fd(2,2,4), fres(2,2,4);
				_interpMap.at(ra->second)->getF(fa);
				_interpMap.at(rb->second)->getF(fb);
				_interpMap.at(rc->second)->getF(fc);
				_interpMap.at(rd->second)->getF(fd);

				// Do interpolation
				matrixop pa(2,2,4), pb(2,2,4), pc(2,2,4), pd(2,2,4);
				pa = fa * ((x2-x)*(y2-y)/denom);
				pb = fb * ((x-x1)*(y2-y)/denom);
				pc = fc * ((x2-x)*(y-y1)/denom);
				pd = fd * ((x-x1)*(y-y1)/denom);
				fres = pa + pb + pc + pd;

				// Convert back to complex doubles and create new daScattMatrix
				// Creating complex types not needed! ddScattMatrix provides the conversion.
				//complex<double> f11(fres.get(2,0,0),fres.get(2,0,1));
				//complex<double> f12(fres.get(2,0,2),fres.get(2,0,3));
				//complex<double> f21(fres.get(2,1,0),fres.get(2,1,1));
				//complex<double> f22(fres.get(2,1,2),fres.get(2,1,3));
				ddscat::ddScattMatrix ress(f,x,y);
				ress.setF(fres);

				// Return result in form of new shared_ptr
				std::shared_ptr<const ddscat::ddScattMatrix> resp(new ddscat::ddScattMatrix(ress));
				res = resp;
				return;
			}

			// Execution should never reach this point.
		}

	} // end interpolation
} // end rtmath

