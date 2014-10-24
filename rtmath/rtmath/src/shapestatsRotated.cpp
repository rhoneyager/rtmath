#include "Stdafx-ddscat.h"
#pragma warning( disable : 4244 ) // annoying double to float issues in boost

#include <functional>
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
#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>

//#include "../rtmath/ddscat/shapestatsRotated.h"
#include "../rtmath/ddscat/shapestats.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/ddscat/hulls.h"

namespace rtmath {
	namespace ddscat {

		namespace stats {

			/* These were removed when I stopped using shapeFileStatsRotated.
			shapeFileStatsRotated::shapeFileStatsRotated() {}
			shapeFileStatsRotated::~shapeFileStatsRotated() {}
			bool shapeFileStatsRotated::operator<(const shapeFileStatsRotated &rhs) const
			{
				if (beta != rhs.beta) return beta < rhs.beta;
				if (theta != rhs.theta) return theta < rhs.theta;
				if (phi != rhs.phi) return phi < rhs.phi;
			}
			*/

			shapeFileStatsBase::rotPtr shapeFileStatsBase::calcStatsRot(double beta, double theta, double phi) const
			{
				//boost::shared_ptr<shapeFileStatsRotated> pres(new shapeFileStatsRotated);
				//shapeFileStatsRotated &res = *pres;
				std::cerr << " " << beta << ", " << theta << ", " << phi << std::endl;

				rotData pres;
				basicTable &tbl = pres.get<0>();
				matrixTable &mat = pres.get<1>();
				vectorTable &vec = pres.get<2>();

				tbl[rotColDefs::BETA] = beta;
				tbl[rotColDefs::THETA] = theta;
				tbl[rotColDefs::PHI] = phi;
				// Please note that the set rotations has a special comparator to allow for 
				// rotational ordering intrusive to a shared pointer, which allows for the 
				// avoidance of a map.
				if (rotstats.count(pres)) // Already calculated
				{
					auto it = rotstats.find(pres);
					// Check if it needs an upgrade
					/// \todo Add easier upgrade code
					if (it->get<0>().at(rotColDefs::VERSION) >= 4) //== shapeFileStatsBase::_maxVersion)
						return it;
					else rotstats.erase(it);
				}


				tbl[rotColDefs::VERSION] = shapeFileStatsBase::_maxVersion;
				Eigen::Matrix3f Roteff;
				rotationMatrix<float>((float)theta, (float)phi, (float)beta, Roteff);
				// Normally, Reff = RyRxRz. But, the rotation is a1,a2-dependent, 
				// which are specified in the file. Apply effective rotation matrix also.

				using namespace boost::accumulators;

				// Figure out potential energy
				// Using moment<1> to rescale value by dividing by the total number of points.
				// I'm currently ignoring the mass term, so am making the ansatz that the whole flake 
				// has unit mass.
				accumulator_set<float, boost::accumulators::stats<tag::min, tag::max, tag::moment<1> > > abs_x, abs_y, abs_z;
				// Tried http://stackoverflow.com/questions/4316716/is-it-possible-to-use-boost-accumulators-with-vectors?rq=1
				// with accumulator_set<vector<double>, ...), but it does not compile on msvc 2010
				typedef accumulator_set<float, boost::accumulators::stats <
					tag::min,
					tag::max,
					tag::moment<1>,
					tag::moment<2>,
					tag::sum,
					tag::skewness,
					tag::kurtosis,
					// Covariances are special
					tag::variance,
					tag::covariance<float, tag::covariate1>,
					tag::covariance < float, tag::covariate2 >
				> > acc_type;
				acc_type acc_x, acc_y, acc_z, acc_r;
				//acc(std::vector<double>(3)); //acc_x, acc_y, acc_z;

				//for (auto it = _shp->latticePtsStd.begin(), ot = _shp->latticePtsRi.begin(); 
				//	it != _shp->latticePtsStd.end(); ++it, ++ot)
				for (size_t i = 0; i < _shp->numPoints; i++)
				{
					auto iit = _shp->latticePts.block<1, 3>(i, 0);
					auto it = &iit; // Laziness when adapting the algorithm

					const Eigen::Vector3f s = (*it).transpose() - b_mean;
					const Eigen::Vector3f pt = Roteff * s;
					const double x = pt(0);
					const double y = pt(1);
					const double z = pt(2);
					acc_x(x, covariate1 = y, covariate2 = z);
					acc_y(y, covariate1 = x, covariate2 = z);
					acc_z(z, covariate1 = x, covariate2 = y);
					acc_r(sqrt((x*x) + (y*y) + (z*z)), covariate1 = x, covariate2 = y);
					abs_x(abs(x));
					abs_y(abs(y));
					abs_z(abs(z));

					// Accumulators are in TF frame? Check against Holly code
				}


#undef min
#undef max
				vec[rotColDefs::MIN](0) = boost::accumulators::min(acc_x);
				vec[rotColDefs::MIN](1) = boost::accumulators::min(acc_y);
				vec[rotColDefs::MIN](2) = boost::accumulators::min(acc_z);
				vec[rotColDefs::MIN](3) = boost::accumulators::min(acc_r);

				vec[rotColDefs::MAX](0) = boost::accumulators::max(acc_x);
				vec[rotColDefs::MAX](1) = boost::accumulators::max(acc_y);
				vec[rotColDefs::MAX](2) = boost::accumulators::max(acc_z);
				vec[rotColDefs::MAX](3) = boost::accumulators::max(acc_r);

				vec[rotColDefs::SUM](0) = boost::accumulators::sum(acc_x);
				vec[rotColDefs::SUM](1) = boost::accumulators::sum(acc_y);
				vec[rotColDefs::SUM](2) = boost::accumulators::sum(acc_z);
				vec[rotColDefs::SUM](3) = boost::accumulators::sum(acc_r);

				vec[rotColDefs::SKEWNESS](0) = boost::accumulators::skewness(acc_x);
				vec[rotColDefs::SKEWNESS](1) = boost::accumulators::skewness(acc_y);
				vec[rotColDefs::SKEWNESS](2) = boost::accumulators::skewness(acc_z);
				vec[rotColDefs::SKEWNESS](3) = boost::accumulators::skewness(acc_r);

				vec[rotColDefs::KURTOSIS](0) = boost::accumulators::kurtosis(acc_x);
				vec[rotColDefs::KURTOSIS](1) = boost::accumulators::kurtosis(acc_y);
				vec[rotColDefs::KURTOSIS](2) = boost::accumulators::kurtosis(acc_z);
				vec[rotColDefs::KURTOSIS](3) = boost::accumulators::kurtosis(acc_r);

				vec[rotColDefs::MOM1](0) = boost::accumulators::moment<1>(acc_x);
				vec[rotColDefs::MOM1](1) = boost::accumulators::moment<1>(acc_y);
				vec[rotColDefs::MOM1](2) = boost::accumulators::moment<1>(acc_z);
				vec[rotColDefs::MOM1](3) = boost::accumulators::moment<1>(acc_r);

				vec[rotColDefs::MOM2](0) = boost::accumulators::moment<2>(acc_x);
				vec[rotColDefs::MOM2](1) = boost::accumulators::moment<2>(acc_y);
				vec[rotColDefs::MOM2](2) = boost::accumulators::moment<2>(acc_z);
				vec[rotColDefs::MOM2](3) = boost::accumulators::moment<2>(acc_r);

				vec[rotColDefs::ABS_MIN](0) = boost::accumulators::min(abs_x); // abs, not acc here
				vec[rotColDefs::ABS_MIN](1) = boost::accumulators::min(abs_y);
				vec[rotColDefs::ABS_MIN](2) = boost::accumulators::min(abs_z);

				vec[rotColDefs::ABS_MAX](0) = boost::accumulators::max(abs_x); // abs, not acc here
				vec[rotColDefs::ABS_MAX](1) = boost::accumulators::max(abs_y);
				vec[rotColDefs::ABS_MAX](2) = boost::accumulators::max(abs_z);

				vec[rotColDefs::ABS_MEAN](0) = boost::accumulators::moment<1>(abs_x); // abs, not acc here
				vec[rotColDefs::ABS_MEAN](1) = boost::accumulators::moment<1>(abs_y);
				vec[rotColDefs::ABS_MEAN](2) = boost::accumulators::moment<1>(abs_z);


				// RMS accumulators - rms is the square root of the 2nd moment - note: this is sample, not pop.
				vec[rotColDefs::RMS_MEAN](0) = sqrt(boost::accumulators::variance(acc_x));
				vec[rotColDefs::RMS_MEAN](1) = sqrt(boost::accumulators::variance(acc_y));
				vec[rotColDefs::RMS_MEAN](2) = sqrt(boost::accumulators::variance(acc_z));
				vec[rotColDefs::RMS_MEAN](3) = sqrt(boost::accumulators::variance(acc_r));

				// Aspect ratios
				// Absolute aspect ratio
				mat[rotColDefs::AS_ABS](0, 0) = 1;
				mat[rotColDefs::AS_ABS](0, 1) = boost::accumulators::max(abs_x) / boost::accumulators::max(abs_y);
				mat[rotColDefs::AS_ABS](0, 2) = boost::accumulators::max(abs_x) / boost::accumulators::max(abs_z);
				mat[rotColDefs::AS_ABS](1, 0) = boost::accumulators::max(abs_y) / boost::accumulators::max(abs_x);
				mat[rotColDefs::AS_ABS](1, 1) = 1;
				mat[rotColDefs::AS_ABS](1, 2) = boost::accumulators::max(abs_y) / boost::accumulators::max(abs_z);
				mat[rotColDefs::AS_ABS](2, 0) = boost::accumulators::max(abs_z) / boost::accumulators::max(abs_x);
				mat[rotColDefs::AS_ABS](2, 1) = boost::accumulators::max(abs_z) / boost::accumulators::max(abs_y);
				mat[rotColDefs::AS_ABS](2, 2) = 1;

				// Absolute mean
				mat[rotColDefs::AS_ABS_MEAN](0, 0) = 1;
				mat[rotColDefs::AS_ABS_MEAN](0, 1) = boost::accumulators::moment<1>(abs_x) / boost::accumulators::moment<1>(abs_y);
				mat[rotColDefs::AS_ABS_MEAN](0, 2) = boost::accumulators::moment<1>(abs_x) / boost::accumulators::moment<1>(abs_z);
				mat[rotColDefs::AS_ABS_MEAN](1, 0) = boost::accumulators::moment<1>(abs_y) / boost::accumulators::moment<1>(abs_x);
				mat[rotColDefs::AS_ABS_MEAN](1, 1) = 1;
				mat[rotColDefs::AS_ABS_MEAN](1, 2) = boost::accumulators::moment<1>(abs_y) / boost::accumulators::moment<1>(abs_z);
				mat[rotColDefs::AS_ABS_MEAN](2, 0) = boost::accumulators::moment<1>(abs_z) / boost::accumulators::moment<1>(abs_x);
				mat[rotColDefs::AS_ABS_MEAN](2, 1) = boost::accumulators::moment<1>(abs_z) / boost::accumulators::moment<1>(abs_y);
				mat[rotColDefs::AS_ABS_MEAN](2, 2) = 1;

				// Absolute rms
				mat[rotColDefs::AS_RMS](0, 0) = 1;
				mat[rotColDefs::AS_RMS](0, 1) = sqrt(boost::accumulators::moment<2>(acc_x) / boost::accumulators::moment<2>(acc_y));
				mat[rotColDefs::AS_RMS](0, 2) = sqrt(boost::accumulators::moment<2>(acc_x) / boost::accumulators::moment<2>(acc_z));
				mat[rotColDefs::AS_RMS](1, 0) = sqrt(boost::accumulators::moment<2>(acc_y) / boost::accumulators::moment<2>(acc_x));
				mat[rotColDefs::AS_RMS](1, 1) = 1;
				mat[rotColDefs::AS_RMS](1, 2) = sqrt(boost::accumulators::moment<2>(acc_y) / boost::accumulators::moment<2>(acc_z));
				mat[rotColDefs::AS_RMS](2, 0) = sqrt(boost::accumulators::moment<2>(acc_z) / boost::accumulators::moment<2>(acc_x));
				mat[rotColDefs::AS_RMS](2, 1) = sqrt(boost::accumulators::moment<2>(acc_z) / boost::accumulators::moment<2>(acc_y));
				mat[rotColDefs::AS_RMS](2, 2) = 1;

				const size_t _N = _shp->numPoints;
				const Eigen::Array3f &d = _shp->d;
				const float dxdydz = d(0) * d(1) * d(2);


				//covariance
				mat[rotColDefs::COVARIANCE](0, 0) = boost::accumulators::variance(acc_x);
				//boost::accumulators::covariance(acc_x, covariate1);
				//boost::accumulators::covariance(acc_x, covariate2);
				mat[rotColDefs::COVARIANCE](0, 1) = boost::accumulators::covariance(acc_x, covariate1);
				mat[rotColDefs::COVARIANCE](0, 2) = boost::accumulators::covariance(acc_x, covariate2);
				mat[rotColDefs::COVARIANCE](1, 0) = boost::accumulators::covariance(acc_y, covariate1);
				mat[rotColDefs::COVARIANCE](1, 1) = boost::accumulators::variance(acc_y);
				mat[rotColDefs::COVARIANCE](1, 2) = boost::accumulators::covariance(acc_y, covariate2);
				mat[rotColDefs::COVARIANCE](2, 0) = boost::accumulators::covariance(acc_z, covariate1);
				mat[rotColDefs::COVARIANCE](2, 1) = boost::accumulators::covariance(acc_z, covariate2);
				mat[rotColDefs::COVARIANCE](2, 2) = boost::accumulators::variance(acc_z);

				// Calculate moments of inertia
				float val = 0;

				// See derivation in Summer 2012 notebook

				// All of these are the partial moments. They need to be scaled by the cell _density_,
				// as multiplying by dxdydz is a volume.
				// I_xx
				val = boost::accumulators::moment<2>(acc_y) +boost::accumulators::moment<2>(acc_z);
				val *= _N * dxdydz;
				mat[rotColDefs::MOMINERT](0, 0) = val;

				// I_yy
				val = boost::accumulators::moment<2>(acc_x) +boost::accumulators::moment<2>(acc_z);
				val *= _N * dxdydz;
				mat[rotColDefs::MOMINERT](1, 1) = val;

				// I_zz
				val = boost::accumulators::moment<2>(acc_x) +boost::accumulators::moment<2>(acc_y);
				val *= _N * dxdydz;
				mat[rotColDefs::MOMINERT](2, 2) = val;

				// I_xy and I_yx
				val = mat[rotColDefs::COVARIANCE](1, 0) + (boost::accumulators::moment<1>(acc_x) * boost::accumulators::moment<1>(acc_y));
				val *= -1.0f * _N * dxdydz;
				mat[rotColDefs::MOMINERT](0, 1) = val;
				mat[rotColDefs::MOMINERT](1, 0) = val;

				// I_xz and I_zx
				val = mat[rotColDefs::COVARIANCE](2, 0) + (boost::accumulators::moment<1>(acc_x) * boost::accumulators::moment<1>(acc_z));
				val *= -1.0f * _N * dxdydz;
				mat[rotColDefs::MOMINERT](0, 2) = val;
				mat[rotColDefs::MOMINERT](2, 0) = val;

				// I_yz and I_zy
				val = mat[rotColDefs::COVARIANCE](2, 1) + (boost::accumulators::moment<1>(acc_y) * boost::accumulators::moment<1>(acc_z));
				val *= -1.0f * _N * dxdydz;
				mat[rotColDefs::MOMINERT](2, 1) = val;
				mat[rotColDefs::MOMINERT](1, 2) = val;

				// Calculate the potential energy
				const float g = 9.80665f; // m/s^2
				// I do not need the partial mass means to be zero here
				vec[rotColDefs::PE](0) = g*boost::accumulators::sum(acc_x);
				vec[rotColDefs::PE](1) = g*boost::accumulators::sum(acc_y);
				vec[rotColDefs::PE](2) = g*boost::accumulators::sum(acc_z);
				vec[rotColDefs::PE](3) = 0; // Writing dir_mat_t, but with three elements instead of four.

				/*
				// Take the voronoi convex hull candidate points, rotate then and project, and find the 2d hull
				auto vcpts = vd->calcCandidateConvexHullPoints();
				//std::cout << *vcpts << std::endl;
				Voronoi::VoronoiDiagram::matrixTypeMutable rpts(new Eigen::MatrixXf(vcpts->rows(), vcpts->cols()));
				rpts->setZero(); // Need to have the final column set to zero?
				for (size_t i = 0; i < (size_t) vcpts->rows(); i++)
				{
					auto iit = vcpts->block<1, 3>(i, 0);
					auto it = &iit; // Laziness when adapting the algorithm

					const Eigen::Vector3f s = (*it).transpose() - b_mean;
					const Eigen::Vector3f pt = Roteff * s;
					const double x = pt(0);
					const double y = pt(1);
					const double z = pt(2);
					
					auto oit = rpts->block<1, 3>(i, 0);
					oit(0) = x;
					oit(1) = y;
					oit(2) = z;
				}
				//std::cout << *rpts << std::endl;
				Voronoi::VoronoiDiagram::matrixType crpts(rpts);

				// for vtk delaunay triangulation, the z coordinate is ignored by default
				// an alpha value may also be specified!

				// using the rotated points
				convexHull cvHull2d(crpts);
				cvHull2d.constructHull();

				*/
				double areas[3] = { 0, 0, 0 }, perims[3] = { 0, 0, 0 };
				///cvHull2d.area2d(areas);
				//cvHull2d.perimeter2d(perims);
				vec[rotColDefs::AREA_CONVEX](0) = static_cast<float>(areas[0]);
				vec[rotColDefs::AREA_CONVEX](1) = static_cast<float>(areas[1]);
				vec[rotColDefs::AREA_CONVEX](2) = static_cast<float>(areas[2]);
				vec[rotColDefs::AREA_CONVEX](3) = 0;

				vec[rotColDefs::PERIMETER_CONVEX](0) = static_cast<float>(perims[0]);
				vec[rotColDefs::PERIMETER_CONVEX](1) = static_cast<float>(perims[1]);
				vec[rotColDefs::PERIMETER_CONVEX](2) = static_cast<float>(perims[2]);
				vec[rotColDefs::PERIMETER_CONVEX](3) = 0;


				// Use std move to insert into set
				//boost::shared_ptr<const shapeFileStatsRotated> cpres
				//	= boost::const_pointer_cast<
				auto r = rotstats.insert(pres);
				return r.first;
			}



		}
	}
}
