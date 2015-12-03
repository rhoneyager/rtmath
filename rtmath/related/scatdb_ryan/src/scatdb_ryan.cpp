#include <memory>
#include <mutex>
#include <string>
#include <iostream>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/covariance.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/kurtosis.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/variates/covariate.hpp>
#include "../scatdb_ryan/scatdb_ryan.hpp"
#include "../scatdb_ryan/lowess.h"
#include "../../spline/spline.hpp"

namespace scatdb_ryan {
	db::db() {}
	db::~db() {}
	scatdb_base::scatdb_base() {}
	scatdb_base::~scatdb_base() {}
	std::shared_ptr<const db::data_stats> db::getStats() const {
		if (!this->pStats)
			this->pStats = db::data_stats::generate(this);
		return this->pStats;
	}
	db::data_stats::data_stats() : count(0) {}
	db::data_stats::~data_stats() {}

	std::shared_ptr<const db> db::interpolate() const {
		std::shared_ptr<db> res(new db);
		// First, get the stats. Want min and max values for effective radius.
		auto stats = this->getStats();
		// TODO: use either aeff or max dimension
		double minRad = stats->floatStats(data_entries::S_MIN,data_entries::AEFF_UM),
			   maxRad = stats->floatStats(data_entries::S_MAX,data_entries::AEFF_UM);

		double low = (double) ((int) (minRad/10.)) * 10.;
		double high = (double) ((int) (maxRad/10.)+1) * 10.;

		// Convert from eigen arrays into vectors
		std::vector<double> aeff, cabs, cbk, cext, csca, g,
			rcabs, rcbk, rcext, rcsca, rg,
			icabs, icbk, icext, icsca, ig,
			rw, residuals, lx, lcabs, lcbk, lcext, lcsca, lg;


		auto convertCol = [&](int col, std::vector<double> &out, bool dolog) {
			auto blk = floatMat.cast<float>().block(0,col,floatMat.rows(),1);
			out.insert(out.begin(), blk.data(), blk.data() + floatMat.rows());
			if (dolog) {
				for (size_t i=0; i< out.size(); ++i)
					out[i] = log10(out[i]);
			}
		};
		convertCol(data_entries::AEFF_UM, aeff, false);
		convertCol(data_entries::CABS_M, cabs, false);
		convertCol(data_entries::CBK_M, cbk, false);
		convertCol(data_entries::CEXT_M, cext, false);
		convertCol(data_entries::CSCA_M, csca, false);
		convertCol(data_entries::G, g, false);

		const int sz = (int)(high-low+1)/10;
		lx.reserve(sz);
		lcabs.reserve(sz);
		lcbk.reserve(sz);
		lcext.reserve(sz);
		lcsca.reserve(sz);
		lg.reserve(sz);

		for (double x = low; x <= high; x += 10) {
			lx.push_back(x);
		}

		// Call interpolation engine here
		//double *spline_cubic_set ( int n, double t[], double y[], int ibcbeg, 
		//	double ybcbeg, int ibcend, double ybcend );
		//double spline_cubic_val ( int n, double t[], double y[], double ypp[], 
		//	double tval, double *ypval, double *yppval );
		auto splineit = [&](std::vector<double> &ys, std::vector<double> &iys) {
			// Really inefficient filtering... Spline code cannot have duplicate x values
			std::vector<double> fxs, fys;
			fxs.reserve(aeff.size()); fys.reserve(aeff.size());
			double last = -9999;
			for (size_t i=0; i<aeff.size(); ++i) {
				double x = aeff[i];
				double y = ys[i];
				if (abs((last/x)-1) < 0.000001 || last >= x) {
					std::cerr << "Ignoring x " << x << " with last " << last << std::endl;
					continue;
				}
				fxs.push_back(x);
				fys.push_back(y);
				last = x;
				//std::cerr << x << std::endl;
			}
			//
			double *aeffpp = spline_cubic_set(fxs.size(), fxs.data(), fys.data(),
				0, 0, 0, 0);


			for (double x = low; x <= high; x += 10) { // duplicates loop from above. change together.
				double ypval = 0, yppval = 0;
				double val = spline_cubic_val(fxs.size(), fxs.data(), fys.data(), aeffpp,
					x, &ypval, &yppval);
				iys.push_back(val);
			}
			delete[] aeffpp;
		};

		splineit(cabs, lcabs);
		splineit(cbk, lcbk);
		splineit(cext, lcext);
		splineit(csca, lcsca);
		splineit(g, lg);

		FloatMatType nfm;
		IntMatType nfi;
		nfm.resize(lx.size(), floatMat.cols());
		nfi.resize(lx.size(), intMat.cols());
		//nfm = floatMat;
		//nfi = intMat;

		nfm.fill(-999);
		nfi.fill(-999);

		//res->intMat.resize(intMat.rows(), intMat.cols());

		auto revertCol = [&](int col, const std::vector<double> &in, bool islog) {
			auto blk = nfm.block(0,col,nfm.rows(),1);
			for (size_t i=0; i<nfm.rows(); ++i) {
				if (!islog)
					blk(i,0) = (float) in[i];
				else
					blk(i,0) = pow(10.f,(float) in[i]);
			}
		};
		revertCol(data_entries::AEFF_UM, lx, false);
		revertCol(data_entries::CABS_M, lcabs, false);
		revertCol(data_entries::CBK_M, lcbk, false);
		revertCol(data_entries::CEXT_M, lcext, false);
		revertCol(data_entries::CSCA_M, lcsca, false);
		revertCol(data_entries::G, lg, false);

		res->floatMat = nfm;
		res->intMat = nfi;
		return res;
	}

	std::shared_ptr<const db> db::regress(double f, long nsteps, double delta) const {
		std::shared_ptr<db> res(new db);

		//nsteps = 2;
		f = 0.1; // 0.1
		delta = 0;
		// First, get the stats. Want min and max values for effective radius.
		auto stats = this->getStats();
		// TODO: use either aeff or max dimension
		double minRad = stats->floatStats(data_entries::S_MIN,data_entries::AEFF_UM),
			   maxRad = stats->floatStats(data_entries::S_MAX,data_entries::AEFF_UM);

		double low = (double) ((int) (minRad/10.)) * 10.;
		double high = (double) ((int) (maxRad/10.)+1) * 10.;

		// Convert from eigen arrays into vectors
		std::vector<double> aeff, cabs, cbk, cext, csca, g,
			rcabs, rcbk, rcext, rcsca, rg,
			icabs, icbk, icext, icsca, ig,
			rw, residuals, lx, lcabs, lcbk, lcext, lcsca, lg;
		auto convertCol = [&](int col, std::vector<double> &out, bool dolog) {
			auto blk = floatMat.cast<float>().block(0,col,floatMat.rows(),1);
			out.insert(out.begin(), blk.data(), blk.data() + floatMat.rows());
			if (dolog) {
				for (size_t i=0; i< out.size(); ++i)
					out[i] = log10(out[i]);
			}
		};
		convertCol(data_entries::AEFF_UM, aeff, false);
		convertCol(data_entries::CABS_M, cabs, true);
		convertCol(data_entries::CBK_M, cbk, true);
		convertCol(data_entries::CEXT_M, cext, true);
		convertCol(data_entries::CSCA_M, csca, true);
		convertCol(data_entries::G, g, false);

		lowess(aeff, cabs, f, nsteps, delta, rcabs, rw, residuals);
		lowess(aeff, cbk, f, nsteps, delta, rcbk, rw, residuals);
		lowess(aeff, cext, f, nsteps, delta, rcext, rw, residuals);
		lowess(aeff, csca, f, nsteps, delta, rcsca, rw, residuals);
		lowess(aeff, g, f, nsteps, delta, rg, rw, residuals);
/*

		*/
		FloatMatType nfm;
		IntMatType nfi;
		//nfm.resize(lx.size(), floatMat.cols());
		//nfi.resize(lx.size(), intMat.cols());
		nfm = floatMat;
		nfi = intMat;

		nfm.block(0,0,nfm.rows(),nfm.cols()).fill(-999);
		nfi.block(0,0,nfi.rows(),nfi.cols()).fill(-999);

		//res->intMat.resize(intMat.rows(), intMat.cols());

		auto revertCol = [&](int col, const std::vector<double> &in, bool islog) {
			auto blk = nfm.block(0,col,nfm.rows(),1);
			for (size_t i=0; i<nfm.rows(); ++i) {
				if (!islog)
					blk(i,0) = (float) in[i];
				else
					blk(i,0) = pow(10.f,(float) in[i]);
			}
		};
		revertCol(data_entries::AEFF_UM, aeff, false);
		revertCol(data_entries::CABS_M, rcabs, true);
		revertCol(data_entries::CBK_M, rcbk, true);
		revertCol(data_entries::CEXT_M, rcext, true);
		revertCol(data_entries::CSCA_M, rcsca, true);
		revertCol(data_entries::G, rg, false);

		res->floatMat = nfm;
		res->intMat = nfi;
		return res;
	}

	std::shared_ptr<const db::data_stats> db::data_stats::generate(const db* src) {
		std::shared_ptr<db::data_stats> res(new db::data_stats);
		if (!src) return res;
		using namespace boost::accumulators;
		typedef accumulator_set<float, boost::accumulators::stats <
			tag::min,
			tag::max,
			tag::mean,
			tag::median,
			tag::skewness,
			tag::kurtosis,
			tag::variance
		> > acc_type;

		// Push the data to the accumulator functions.
		std::vector<acc_type> accs(data_entries::NUM_DATA_ENTRIES_FLOATS);
		for (int i=0; i<src->floatMat.rows(); ++i) {
			for (int j=0; j<src->floatMat.cols(); ++j) {
				if (src->floatMat(i,j) < -900) continue;
				accs[j](src->floatMat(i,j));
			}
		}
		res->count = src->floatMat.rows();

		// Extract the parameters
#undef min
#undef max
		//typedef Eigen::Matrix<float, data_entries::NUM_DATA_ENTRIES_STATS,
		//	data_entries::NUM_DATA_ENTRIES_FLOATS> StatsFloatType;
		for (int j=0; j<data_entries::NUM_DATA_ENTRIES_FLOATS; ++j) {
				res->floatStats(data_entries::S_MIN,j) = boost::accumulators::min(accs[j]);
				res->floatStats(data_entries::S_MAX,j) = boost::accumulators::max(accs[j]);
				res->floatStats(data_entries::MEAN,j) = boost::accumulators::mean(accs[j]);
				res->floatStats(data_entries::MEDIAN,j) = boost::accumulators::median(accs[j]);
				res->floatStats(data_entries::SKEWNESS,j) = boost::accumulators::skewness(accs[j]);
				res->floatStats(data_entries::KURTOSIS,j) = boost::accumulators::kurtosis(accs[j]);
				res->floatStats(data_entries::SD,j) = std::pow(boost::accumulators::variance(accs[j]),0.5);
		}
		return res;
	}

	template<>
	const char* db::data_entries::stringify<float>(int ev) {
#define _tostr(a) #a
#define tostr(a) _tostr(a)
#define check(a) if (ev == a) return tostr(a);
		check(FREQUENCY_GHZ); check(TEMPERATURE_K); check(AEFF_UM);
		check(MAX_DIMENSION_MM); check(CABS_M); check(CBK_M); check(CEXT_M);
		check(CSCA_M); check(G); check(AS_XY);

		return "";
	}

	template<>
	const char* db::data_entries::stringify<int>(int ev) {
		check(FLAKETYPE);
		return "";
	}

	void db::data_stats::print(std::ostream &out) const {
		out << "Stats for " << count << " entries." << std::endl;
		out << "\tMIN\tMAX\tMEDIAN\tMEAN\tSD\tSKEWNESS\tKURTOSIS\n";
		for (int i=0; i<data_entries::NUM_DATA_ENTRIES_FLOATS; ++i) {
			out << data_entries::stringify<float>(i) << std::endl;
			out << "\t" << floatStats(data_entries::S_MIN,i) << "\t"
				<< floatStats(data_entries::S_MAX,i) << "\t"
				<< floatStats(data_entries::MEDIAN,i) << "\t"
				<< floatStats(data_entries::MEAN,i) << "\t"
				<< floatStats(data_entries::SD,i) << "\t"
				<< floatStats(data_entries::SKEWNESS,i) << "\t"
				<< floatStats(data_entries::KURTOSIS,i) << std::endl;
		}
	}
}

