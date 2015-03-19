#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4244 ) // annoying double to float issues in boost

#include <set>
#include <boost/math/constants/constants.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>
#include <limits>
#include <Eigen/Dense>

#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/common_templates.h"
#include "../rtmath/config.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

#include "../rtmath/ddscat/hulls.h"

#include "shapestats_private.h"


#undef min
#undef max

namespace rtmath {
	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::ddscat::stats::shapeFileStats>;

		template struct IO_class_registry_reader
			<::rtmath::ddscat::stats::shapeFileStats>;

		template class usesDLLregistry<
			::rtmath::ddscat::stats::shapeFileStats_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::stats::shapeFileStats> >;

		template class usesDLLregistry<
			::rtmath::ddscat::stats::shapeFileStats_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::stats::shapeFileStats> >;

	}
	namespace ddscat {
		namespace stats {

			
			void shapeFileStatsBase::calcStatsBaseRotMatrix()
			{
				const Eigen::Array3f &a1 = _shp->a1;
				const Eigen::Array3f &a2 = _shp->a2;
				const Eigen::Array3f &a3 = _shp->a3;
				// Figure out the base rotation from a1 = <1,0,0>, a2 = <0,1,0> that 
				// gives the current a1, a2.

				float thetar, betar, phir;
				// Theta is the angle between a1 and xlf
				// From dot product, theta = acos(a1.xlf)
				float dp = a1(0); // need only x component, as xlf is the unit vector in +x
				thetar = acos(dp);

				// From a1 = x_lf*cos(theta) + y_lf*sin(theta)*cos(phi) + z_lf*sin(theta)*sin(phi),
				// can use either y_lf or z_lf components to get phi
				float stheta = sin(thetar);
				if (thetar)
				{
					float acphi = a1(1) / stheta;
					phir = acos(acphi);

					// Finally, a2_x = -sin(theta)cos(beta)
					float cbeta = a2(0) / stheta * -1.f;
					betar = acos(cbeta);
				}
				else {
					// theta is zero, so gimbal locking occurs. assume phi = 0.
					phir = 0;
					// must use alternate definition to get beta
					float cosbeta = a2(1);
					betar = acos(cosbeta);
				}

				// thetar, betar, phir are in radians
				// convert to degrees
				{
					double scale = 180.0 / (boost::math::constants::pi<double>());
					beta = betar * scale;
					theta = thetar * scale;
					phi = phir * scale;
				}

				rotationMatrix<float>(theta, phi, beta, rot);
				invrot = rot.inverse();

			}

			void shapeFileStatsBase::calcBs()
			{
				// Define statistics for max, min, mean, std dev, skewness, kurtosis, moment of inertia
				using namespace boost::accumulators;
				//using namespace boost::accumulators::tag;

				// Do two passes to be able to renormalize coordinates
				accumulator_set<float, boost::accumulators::stats<tag::mean, tag::min, tag::max> > m_x, m_y, m_z;
				//for (auto it = _shp->latticePtsStd.begin(); it != _shp->latticePtsStd.end(); ++it)
				for (size_t i = 0; i < _shp->numPoints; i++)
				{
					auto it = _shp->latticePts.block<1, 3>(i, 0);
					m_x((it)(0));
					m_y((it)(1));
					m_z((it)(2));
				}

				b_min(0) = boost::accumulators::min(m_x);
				b_min(1) = boost::accumulators::min(m_y);
				b_min(2) = boost::accumulators::min(m_z);

				b_max(0) = boost::accumulators::max(m_x);
				b_max(1) = boost::accumulators::max(m_y);
				b_max(2) = boost::accumulators::max(m_z);

				b_mean(0) = boost::accumulators::mean(m_x);
				b_mean(1) = boost::accumulators::mean(m_y);
				b_mean(2) = boost::accumulators::mean(m_z);
			}

			void shapeFileStatsBase::calcVoroCvx()
			{

				// Using the convex hull to get the maximum diameter
				using namespace rtmath::Voronoi;
				std::cerr << " Voronoi" << std::endl;
				boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram> vd;
				// Voronoi diagram is used twice - to calcuate voronoi stats and to 
				// prefilter the points for the convex hull stats.
				//boost::shared_ptr<VoronoiDiagram> vd;
				if (doVoronoi)
				{
					vd = _shp->generateVoronoi(
						std::string("standard"), VoronoiDiagram::generateStandard);
				}

				boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >
					candidate_hull_points;
				if (_shp->latticeExtras.count("cvxcands"))
				{
					candidate_hull_points = _shp->latticeExtras["cvxcands"];
				}
				else {
					if (doVoronoi)
						candidate_hull_points = vd->calcCandidateConvexHullPoints();
					else {
						boost::shared_ptr
							< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > m
							(new Eigen::MatrixXf(_shp->numPoints, 3));
						*m = _shp->latticePtsNorm;
						candidate_hull_points = m;
					}
				}
				std::cerr << " Voronoi diagram calculated. Extracting " << candidate_hull_points->rows() << " hull points." << std::endl;
				boost::shared_ptr<convexHull> cvHull(convexHull::generate(candidate_hull_points));
				cvHull->constructHull();
				max_distance = cvHull->maxDiameter();

				// Calculate stats on max aspect ratio
				/*{
				double beta, theta, phi;
				cvHull.principalAxes(beta, theta, phi);
				_rotMaxAR = calcStatsRot(beta, theta, phi);
				}*/


				vd->calcSurfaceDepth();
				size_t numLatticeTotal = 0, numLatticeFilled = 0;
				size_t int_voro_depth = 2;
				vd->calcFv(int_voro_depth, numLatticeTotal, numLatticeFilled);
				SVoronoi_internal_2.f = (double)numLatticeFilled / (double)numLatticeTotal;
				SVoronoi_internal_2.aeff_V = aeff_dipoles_const;
				SVoronoi_internal_2.aeff_SA = SVoronoi_internal_2.aeff_V;
				SVoronoi_internal_2.V = boost::math::constants::pi<float>() * 4.0f * pow(SVoronoi_internal_2.aeff_V, 3.0f) / 3.0f;
				SVoronoi_internal_2.SA = boost::math::constants::pi<float>() * 4.0f * pow(SVoronoi_internal_2.aeff_SA, 2.0f);
				//SVoronoi_internal_2.calc(this);
				auto voroHullCalc = [&]()
				{
					std::pair<float, float> res;
					if (doVoronoi)
					{
						res.first = (float)vd->volume();
						res.second = (float)vd->surfaceArea();
					}
					else {
						res.first = -1.f;
						res.second = -1.f;
					}
					return res;
				};

				auto cvxHullCalc = [&]()
				{
					std::pair<float, float> res;
					res.first = (float)cvHull->volume();
					res.second = (float)cvHull->surfaceArea();
					return res;
				};

				Sconvex_hull.calc(this, cvxHullCalc);
				SVoronoi_hull.calc(this, voroHullCalc);

			}

			void shapeFileStatsBase::calcScircum()
			{
				Scircum_sphere.aeff_V = max_distance / 2.0;
				Scircum_sphere.aeff_SA = max_distance / 2.0;
				Scircum_sphere.V = boost::math::constants::pi<float>() * 4.0f * pow(Scircum_sphere.aeff_V, 3.0f) / 3.0f;
				Scircum_sphere.SA = boost::math::constants::pi<float>() * 4.0f * pow(Scircum_sphere.aeff_SA, 2.0f);
				Scircum_sphere.calc(this);
			}

			void shapeFileStatsBase::calcSsolid()
			{
				Ssolid.aeff_V = aeff_dipoles_const;
				Ssolid.aeff_SA = aeff_dipoles_const;
				Ssolid.V = V_dipoles_const; //boost::math::constants::pi<float>() * 4.0f * pow(Ssolid.aeff_V, 3.0f) / 3.0f;
				Ssolid.SA = boost::math::constants::pi<float>() * 4.0f * pow(Ssolid.aeff_SA, 2.0f);
				Ssolid.calc(this);
			}

			void shapeFileStatsBase::calcSellmax()
			{
				auto pdr = calcStatsRot(0, 0, 0);
				const basicTable &tbl = pdr->get<0>();
				const matrixTable &mat = pdr->get<1>();
				const vectorTable &vec = pdr->get<2>();


				Sellipsoid_max.V = boost::math::constants::pi<float>() / 6.0f;
				// Using diameters, and factor in prev line reflects this

				/// \todo Check V_ellipsoid_max and V_ellipsoid_rms calculations!
				Sellipsoid_max.V *= vec[rotColDefs::MAX](0) - vec[rotColDefs::MIN](0);
				Sellipsoid_max.V *= vec[rotColDefs::MAX](1) - vec[rotColDefs::MIN](1);
				Sellipsoid_max.V *= vec[rotColDefs::MAX](2) - vec[rotColDefs::MIN](2);

				//Sellipsoid_rms.V = 4.0f * boost::math::constants::pi<float>() / 3.0f;
				//Sellipsoid_rms.V *= pdr->max(0,0) - pdr->min(0,0);
				//Sellipsoid_rms.V *= pdr->max(0,1) - pdr->min(0,1);
				//Sellipsoid_rms.V *= pdr->max(0,2) - pdr->min(0,2);

				Sellipsoid_max.calc(this);
			}

			void shapeFileStatsBase::calcSrms_sphere()
			{
				auto pdr = calcStatsRot(0, 0, 0);
				const basicTable &tbl = pdr->get<0>();
				const matrixTable &mat = pdr->get<1>();
				const vectorTable &vec = pdr->get<2>();

				// RMS aeff is determined by summation over all points to get the rms vector.
				// Can't use RMS_MEAN, since the mean should be zero, not the mean of the r distribution.
				// Also, RMS_MEAN is for a sample, not a population.
				double rms_index = vec[rotColDefs::MOM2](0) + vec[rotColDefs::MOM2](1) + vec[rotColDefs::MOM2](2);
				rms_index *= 5. / 3.;
				//rms_index *= sqrt(5. / (3. * (double)_shp->numPoints));
				//double rms_index = vec[rotColDefs::RMS_MEAN](3) * sqrt(5. / 3.);
				Srms_sphere.aeff_V = sqrt(rms_index);
				Srms_sphere.aeff_SA = sqrt(rms_index);
				Srms_sphere.V = boost::math::constants::pi<float>() * 4.0f * pow(Srms_sphere.aeff_V, 3.0f) / 3.0f;
				Srms_sphere.SA = boost::math::constants::pi<float>() * 4.0f * pow(Srms_sphere.aeff_SA, 2.0f);
				Srms_sphere.calc(this);
			}

			void shapeFileStatsBase::calcSgyration()
			{
				auto pdr = calcStatsRot(0, 0, 0);
				const basicTable &tbl = pdr->get<0>();
				const matrixTable &mat = pdr->get<1>();
				const vectorTable &vec = pdr->get<2>();

				/// need r^2, not r, so MOM2, not SUM
				//double g_index = vec[rotColDefs::MOM2](0) + vec[rotColDefs::MOM2](1) + vec[rotColDefs::MOM2](2);
				//g_index /= (double)_shp->numPoints; ?
				//Sgyration.aeff_V = pow(g_index, 2.) / (double)_shp->numPoints; // / (double)_shp->numPoints, 2.);
				double rms_index = vec[rotColDefs::MOM2](0) + vec[rotColDefs::MOM2](1) + vec[rotColDefs::MOM2](2);
				Sgyration.aeff_V = sqrt(rms_index);
				Sgyration.aeff_SA = Sgyration.aeff_V;
				Sgyration.V = boost::math::constants::pi<float>() * 4.0f * pow(Sgyration.aeff_V, 3.0f) / 3.0f;
				Sgyration.SA = boost::math::constants::pi<float>() * 4.0f * pow(Sgyration.aeff_SA, 2.0f);
				Sgyration.calc(this);
			}

			void shapeFileStatsBase::calcStatsBase()
			{
				using namespace std;
				// Do calculations of the center of mass, the tensor quantities, and other stuff
				// The functions called here are all indep. of the initial state, as mass, density,
				// volume and everything else have been calculated already.

				// Define the accumulators that we want
				// For each axis, get min, max and the other statistics about the distribution

				std::cerr << "Calculating base stats for shape with hash " << _shp->hash().string() << std::endl;
				// Iterate accumulator as function of radial distance from center of mass

				// Pull in some vars from the shapefile
				const size_t _N = _shp->numPoints;
				std::cerr << " " << _N << " points" << std::endl;

				if (!_N)
					RTthrow(rtmath::debug::xBadInput())
						<< rtmath::debug::otherErrorText("Stats cannot be calculated because the shapefile is not loaded.");

				// Calculate volume elements
				float dxdydz = _shp->d(0) * _shp->d(1) * _shp->d(2);
				V_cell_const = dxdydz;
				V_dipoles_const = dxdydz * _N;
				aeff_dipoles_const = pow(V_dipoles_const*3.f/(4.f*boost::math::constants::pi<float>()),1.f/3.f);
				
				calcStatsBaseRotMatrix();
				calcBs();

				calcSsolid();
				calcVoroCvx();
				calcScircum();
				

				std::cerr << " Calculating 0,0,0 rotation" << std::endl;
				// Calculate rotated stats to avoid having to duplicate code
				// From the 0,0,0 rotation,
				calcSellmax();
				calcSrms_sphere();
				calcSgyration();


				// Calculate all default (from config or command-line) rotations
				for (auto rot : defaultRots)
				{
					//std::cerr << "Calculating stats for rotation " << rot.get<0>() << ", " << rot.get<1>() << ", " << rot.get<2>() << std::endl;
					calcStatsRot(rot.get<0>(), rot.get<1>(), rot.get<2>());
					//const basicTable &tbl = rot.get<0>();
					//calcStatsRot(tbl[rotColDefs::BETA], tbl[rotColDefs::THETA], tbl[rotColDefs::PHI]);
				}
				//calcOriMinPE();
			}



		}
	}
}

