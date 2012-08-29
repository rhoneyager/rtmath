#include "../rtmath/Stdafx.h"

#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/shapestatsRotated.h"
#include "../rtmath/ddscat/shapestats.h"

namespace rtmath {
	namespace ddscat {

		void shapeFileStatsBase::calcStatsRot(double beta, double theta, double phi)
		{
			shapeFileStatsRotated res;
			res.beta = beta;
			res.theta = theta;
			res.phi = phi;
			if (rotations.count(res)) return; // Already calculated

			const double drconv = 2.0*boost::math::constants::pi<double>()/180.0;
			double cb = cos(beta*drconv);
			double ct = cos(theta*drconv);
			double cp = cos(phi*drconv);
			double sb = sin(beta*drconv);
			double st = sin(theta*drconv);
			double sp = sin(phi*drconv);
			// Do right-handed rotation
			// It's just easier to express the overall rotation as the multiplication of
			// the component Gimbal matrices.
			matrixop Rx(2,3,3), Ry(2,3,3), Rz(2,3,3);

			Rx.set(1,2,0,0);
			Rx.set(cp,2,1,1);
			Rx.set(cp,2,2,2);
			Rx.set(sp,2,2,1);
			Rx.set(-sp,2,1,2);

			Ry.set(cb,2,0,0);
			Ry.set(1 ,2,1,1);
			Ry.set(cb,2,2,2);
			Ry.set(sb,2,0,2);
			Ry.set(-sb,2,2,0);

			Rz.set(ct,2,0,0);
			Rz.set(ct,2,1,1);
			Rz.set(1,2,2,2);
			Rz.set(st,2,1,0);
			Rz.set(-st,2,0,1);

			// Normally, Reff = RyRxRz. But, the rotation is a1,a2-dependent, 
			// which are specified in the file. Apply effective rotation matrix also.
			matrixop Roteff = Ry*Rx*Rz*rot;

			using namespace boost::accumulators;

			// Figure out potential energy
			// Using moment<1> to rescale value by dividing by the total number of points.
			// I'm currently ignoring the mass term, so am making the ansatz that the whole flake 
			// has unit mass.
			accumulator_set<double, stats<tag::min, tag::max, tag::moment<1> > > abs_x, abs_y, abs_z;
			// Tried http://stackoverflow.com/questions/4316716/is-it-possible-to-use-boost-accumulators-with-vectors?rq=1
			// with accumulator_set<vector<double>, ...), but it does not compile on msvc 2010
			std::vector<accumulator_set<double, stats<
				tag::min,
				tag::max, 
				tag::moment<1>,
				tag::moment<2>,
				tag::sum,
				tag::skewness,
				tag::kurtosis,
				// Covariances are special
				tag::variance,
				tag::covariance<double, tag::covariate1>,
				tag::covariance<double, tag::covariate2>
			> > > acc_x, acc_y, acc_z; //acc(std::vector<double>(3)); //acc_x, acc_y, acc_z;

			const size_t nV = _shp->_Dielectrics.size()+1;
			matrixop cloner(2,3,1), clonerb(2,3,3);
			acc_x.resize(nV); // Assumes that the dielectrics start at 1.
			acc_y.resize(nV); // Will probably crash if not.
			acc_z.resize(nV);
			res.PE.resize(nV, cloner);
			res.mom1.resize(nV, cloner);
			res.mom2.resize(nV, cloner);
			res.mominert.resize(nV,clonerb);
			res.covariance.resize(nV,clonerb);

			for (auto it = _shp->_latticePtsStd.begin(), ot = _shp->_latticePtsRi.begin(); 
				it != _shp->_latticePtsStd.end(); it++, ot++)
			{
				// it->first is the points id. it->second is its matrixop coords (1x3 matrix)
				// Mult by rotaion matrix to get 3x1 rotated matrix

				matrixop pt = Roteff * (it->transpose() - b_mean);
				double x = pt.get(2,0,0);
				double y = pt.get(2,1,0);
				double z = pt.get(2,2,0);
				size_t diel = ot->get(2,0,0);

				//vector<double> vpt(3);
				//pt.to<std::vector<double> >(vpt);
				acc_x[0](x, covariate1 = y, covariate2 = z);
				acc_y[0](y, covariate1 = x, covariate2 = z);
				acc_z[0](z, covariate1 = x, covariate2 = y);
				acc_x[diel](x, covariate1 = y, covariate2 = z);
				acc_y[diel](y, covariate1 = x, covariate2 = z);
				acc_z[diel](z, covariate1 = x, covariate2 = y);
				abs_x(abs(x));
				abs_y(abs(y));
				abs_z(abs(z));

				// Accumulators are in TF frame? Check against Holly code
			}

			// Are other quantities needed?

			// Export to class matrixops
			res.min.set(boost::accumulators::min(acc_x[0]),2,0,0);
			res.min.set(boost::accumulators::min(acc_y[0]),2,1,0);
			res.min.set(boost::accumulators::min(acc_z[0]),2,2,0);

			res.max.set(boost::accumulators::max(acc_x[0]),2,0,0);
			res.max.set(boost::accumulators::max(acc_y[0]),2,1,0);
			res.max.set(boost::accumulators::max(acc_z[0]),2,2,0);

			res.sum.set(boost::accumulators::sum(acc_x[0]),2,0,0);
			res.sum.set(boost::accumulators::sum(acc_y[0]),2,1,0);
			res.sum.set(boost::accumulators::sum(acc_z[0]),2,2,0);

			res.skewness.set(boost::accumulators::skewness(acc_x[0]),2,0,0);
			res.skewness.set(boost::accumulators::skewness(acc_y[0]),2,1,0);
			res.skewness.set(boost::accumulators::skewness(acc_z[0]),2,2,0);

			res.kurtosis.set(boost::accumulators::kurtosis(acc_x[0]),2,0,0);
			res.kurtosis.set(boost::accumulators::kurtosis(acc_y[0]),2,1,0);
			res.kurtosis.set(boost::accumulators::kurtosis(acc_z[0]),2,2,0);


			// Absolue value-dependent quantities

			res.abs_min.set(boost::accumulators::min(abs_x),2,0,0); // abs, not acc here
			res.abs_min.set(boost::accumulators::min(abs_y),2,1,0);
			res.abs_min.set(boost::accumulators::min(abs_z),2,2,0);

			res.abs_max.set(boost::accumulators::max(abs_x),2,0,0); // abs, not acc here
			res.abs_max.set(boost::accumulators::max(abs_y),2,1,0);
			res.abs_max.set(boost::accumulators::max(abs_z),2,2,0);

			res.abs_mean.set(boost::accumulators::moment<1>(abs_x),2,0,0); // abs, not acc here
			res.abs_mean.set(boost::accumulators::moment<1>(abs_y),2,1,0);
			res.abs_mean.set(boost::accumulators::moment<1>(abs_z),2,2,0);

			// Aspect ratios
			// Absolute aspect ratio
			res.as_abs.set(1,2,0,0);
			res.as_abs.set(boost::accumulators::max(abs_x)/boost::accumulators::max(abs_y),2,0,1);
			res.as_abs.set(boost::accumulators::max(abs_x)/boost::accumulators::max(abs_z),2,0,2);
			res.as_abs.set(boost::accumulators::max(abs_y)/boost::accumulators::max(abs_x),2,1,0);
			res.as_abs.set(1,2,1,1);
			res.as_abs.set(boost::accumulators::max(abs_y)/boost::accumulators::max(abs_z),2,1,2);
			res.as_abs.set(boost::accumulators::max(abs_z)/boost::accumulators::max(abs_x),2,2,0);
			res.as_abs.set(boost::accumulators::max(abs_z)/boost::accumulators::max(abs_y),2,2,1);
			res.as_abs.set(1,2,2,2);

			// Absolute mean
			res.as_abs_mean.set(1,2,0,0);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_x)/boost::accumulators::moment<1>(abs_y),2,0,1);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_x)/boost::accumulators::moment<1>(abs_z),2,0,2);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_y)/boost::accumulators::moment<1>(abs_x),2,1,0);
			res.as_abs_mean.set(1,2,1,1);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_y)/boost::accumulators::moment<1>(abs_z),2,1,2);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_z)/boost::accumulators::moment<1>(abs_x),2,2,0);
			res.as_abs_mean.set(boost::accumulators::moment<1>(abs_z)/boost::accumulators::moment<1>(abs_y),2,2,1);
			res.as_abs_mean.set(1,2,2,2);

			// Absolute rms
			res.as_rms.set(1,2,0,0);
			res.as_rms.set(sqrt(boost::accumulators::moment<2>(acc_x[0])/boost::accumulators::moment<2>(acc_y[0])),2,0,1);
			res.as_rms.set(sqrt(boost::accumulators::moment<2>(acc_x[0])/boost::accumulators::moment<2>(acc_z[0])),2,0,2);
			res.as_rms.set(sqrt(boost::accumulators::moment<2>(acc_y[0])/boost::accumulators::moment<2>(acc_x[0])),2,1,0);
			res.as_rms.set(1,2,1,1);
			res.as_rms.set(sqrt(boost::accumulators::moment<2>(acc_y[0])/boost::accumulators::moment<2>(acc_z[0])),2,1,2);
			res.as_rms.set(sqrt(boost::accumulators::moment<2>(acc_z[0])/boost::accumulators::moment<2>(acc_x[0])),2,2,0);
			res.as_rms.set(sqrt(boost::accumulators::moment<2>(acc_z[0])/boost::accumulators::moment<2>(acc_y[0])),2,2,1);
			res.as_rms.set(1,2,2,2);

			// RMS accumulators - rms is the square root of the 2nd moment
			res.rms_mean.set(sqrt(boost::accumulators::variance(acc_x[0])),2,0,0);
			res.rms_mean.set(sqrt(boost::accumulators::variance(acc_y[0])),2,1,0);
			res.rms_mean.set(sqrt(boost::accumulators::variance(acc_z[0])),2,2,0);

			const size_t _N = _shp->_numPoints;
			const matrixop &d = _shp->_d;
			const double dxdydz = d.get(2,0,0) * d.get(2,0,1) * d.get(2,0,2);

			for (size_t i=0; i<nV; i++)
			{
				res.mom1[i].set(boost::accumulators::moment<1>(acc_x[i]),2,0,0);
				res.mom1[i].set(boost::accumulators::moment<1>(acc_y[i]),2,1,0);
				res.mom1[i].set(boost::accumulators::moment<1>(acc_z[i]),2,2,0);

				res.mom2[i].set(boost::accumulators::moment<2>(acc_x[i]),2,0,0);
				res.mom2[i].set(boost::accumulators::moment<2>(acc_y[i]),2,1,0);
				res.mom2[i].set(boost::accumulators::moment<2>(acc_z[i]),2,2,0);

				//covariance
				res.covariance[i].set(boost::accumulators::variance(acc_x[i]),2,0,0);
				//boost::accumulators::covariance(acc_x, covariate1);
				//boost::accumulators::covariance(acc_x, covariate2);
				res.covariance[i].set(boost::accumulators::covariance(acc_x[i], covariate1),2,0,1);
				res.covariance[i].set(boost::accumulators::covariance(acc_x[i], covariate2),2,0,2);
				res.covariance[i].set(boost::accumulators::covariance(acc_y[i], covariate1),2,1,0);
				res.covariance[i].set(boost::accumulators::variance(acc_y[i]),2,1,1);
				res.covariance[i].set(boost::accumulators::covariance(acc_y[i], covariate2),2,1,2);
				res.covariance[i].set(boost::accumulators::covariance(acc_z[i], covariate1),2,2,0);
				res.covariance[i].set(boost::accumulators::covariance(acc_z[i], covariate2),2,2,1);
				res.covariance[i].set(boost::accumulators::variance(acc_z[i]),2,2,2);

				// Calculate moments of inertia
				double val = 0;
				// TODO: All wrong. Need to redo.
				// I_xx
				val = boost::accumulators::variance(acc_y[i]) + boost::accumulators::variance(acc_z[i]);
				val *= _N * dxdydz;
				res.mominert[i].set(val,2,0,0);

				// I_yy
				val = boost::accumulators::variance(acc_x[i]) + boost::accumulators::variance(acc_z[i]);
				val *= _N * dxdydz;
				res.mominert[i].set(val,2,1,1);

				// I_zz
				val = boost::accumulators::variance(acc_x[i]) + boost::accumulators::variance(acc_y[i]);
				val *= _N * dxdydz;
				res.mominert[i].set(val,2,2,2);

				// I_xy and I_yx
				val = -1.0 * res.covariance[i].get(2,1,0);
				val *= _N * dxdydz;
				res.mominert[i].set(val,2,0,1);
				res.mominert[i].set(val,2,1,0);

				// I_xz and I_zx
				val = -1.0 * res.covariance[i].get(2,2,0);
				val *= _N * dxdydz;
				res.mominert[i].set(val,2,0,2);
				res.mominert[i].set(val,2,2,0);

				// I_yz and I_zy
				val = -1.0 * res.covariance[i].get(2,2,1);
				val *= _N * dxdydz;
				res.mominert[i].set(val,2,2,1);
				res.mominert[i].set(val,2,1,2);
			}

			// Are other quantities needed?


			// Use std move to insert into set
			rotations.insert(std::move(res));
		}


		shapeFileStatsRotated::shapeFileStatsRotated()
			: min(2,3,1), max(2,3,1), sum(2,3,1), skewness(2,3,1), kurtosis(2,3,1),
			abs_min(2,3,1), abs_max(2,3,1), abs_mean(2,3,1),
			as_abs(2,3,3), as_abs_mean(2,3,3), as_rms(2,3,3),
			rms_mean(2,3,1)
		{
			this->beta = 0;
			this->theta = 0;
			this->phi = 0;
		}

		shapeFileStatsRotated::~shapeFileStatsRotated()
		{
		}

		bool shapeFileStatsRotated::operator<(const shapeFileStatsRotated &rhs) const
		{
			if (beta!=rhs.beta) return beta<rhs.beta;
			if (theta!=rhs.theta) return theta<rhs.theta;
			if (phi!=rhs.phi) return phi<rhs.phi;
			return false;
		}

	}
}
