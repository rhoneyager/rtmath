#pragma once
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "matrixop.h"
#include "coords.h"
#include "ddscat/ddScattMatrix.h"
//#include "ddscat/ddOutputEnsemble.h"
//#include "ddscat/ddOutputSingle.h"

/* interpolatable is an abstract base class that provides some functionality for
   interpolating sets of objects based on their coordinates. A good use of this
   is ddOutputSingle. ddStatic works too. The base class allows for specification
   of the type of interpolation (lineal, bicubic, etc.), and the functions 
   provised provide a set of weights associated with each coordinate.
   */

namespace rtmath {

	namespace interpolation {
		// Seceral possible interpolation classes may exist, including templates.
		// depending on input type, so it becomes convenient to have these all 
		// under the same namespace.

		enum interpMethod
		{
			NEAREST_NEIGHBOR,
			BILINEAR,
			UNKNOWN
		};

		class ddOutputSingleInterp
		{
		public:

			ddOutputSingleInterp();
			virtual ~ddOutputSingleInterp();
			virtual void interpolate(
				const coords::cyclic<double> &pt,
				std::shared_ptr<const ddscat::ddScattMatrix> &res
				) const;
			
		protected:

			virtual void _insert(std::shared_ptr<const ddscat::ddScattMatrix> &obj);
			virtual void _clear();

		public:

			std::unordered_map<
				coords::cyclic<double>, 
				std::shared_ptr<const ddscat::ddScattMatrix>, 
				boost::hash<coords::cyclic<double> > 
				> _interpMap;

			mutable std::set<std::shared_ptr<const ddscat::ddScattMatrix> > _scattMatricesRaw;
			mutable std::set<std::shared_ptr<const ddscat::ddScattMatrix> > _scattMatricesAll;

			void _init();
			interpMethod _interpMethod;

			size_t _sizeRaw() const;
			size_t _sizeRawNan() const;

			/*friend class ddscat::ddOutputEnsemble;
			friend class ddscat::ddOutputEnsembleGaussian;
			friend class ddscat::ddOutputEnsembleIsotropic;*/
		};

		// TODO: implement a very general method later
		/*
		template <class T, class Q>
		class interpolatable
		{
		public:
			interpolatable(interpMethod method = UNKNOWN)
			{
				_interpMethod = method;
			}
			virtual ~interpolatable();
			virtual void interpolate(
				const coords::cyclic<T> &pt,
				Q &res
				) const;
		protected:
			interpMethod _interpMethod;
			mutable std::unordered_map<coords::cyclic<double>, std::shared_ptr<const ddScattMatrix>, 
				boost::hash<coords::cyclic<double> > > _interpMap;
		private:
			void _init();
		}
		*/


		/*

			virtual void interpolate(const coords::cyclic<double> &pt,
				std::unordered_map<coords::cyclic<double> , 
					double, boost::hash<coords::cyclic<double> > > &weights);

		*/


	} // end interpolation namespace
} // end rtmath


