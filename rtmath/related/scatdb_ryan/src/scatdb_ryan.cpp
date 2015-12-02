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

namespace scatdb_ryan {
	db::db() {}
	db::~db() {}
	scatdb_base::scatdb_base() {}
	scatdb_base::~scatdb_base() {}
	std::shared_ptr<const db::data_stats> db::getStats() const {
		return db::data_stats::generate(this);
	}
	db::data_stats::data_stats() : count(0) {}
	db::data_stats::~data_stats() {}

	std::shared_ptr<const db> db::regress(double f, long nsteps, double delta) const {
		std::shared_ptr<db> res(new db);

		//nsteps = 2;
		f = 0.1;
		delta = 0;
		// First, get the stats. Want min and max values for effective radius.
		auto stats = this->getStats();
		double minRad = stats->floatStats(data_entries::S_MIN,data_entries::AEFF_UM),
			   maxRad = stats->floatStats(data_entries::S_MAX,data_entries::AEFF_UM);

		// Convert from eigen arrays into vectors
		std::vector<double> aeff, cabs, cbk, cext, csca, g,
			rcabs, rcbk, rcext, rcsca, rg,
			icabs, icbk, icext, icsca, ig,
			rw, residuals;
		auto convertCol = [&](int col, std::vector<double> &out) {
			auto blk = floatMat.cast<float>().block(0,col,floatMat.rows(),1);
			out.insert(out.begin(), blk.data(), blk.data() + floatMat.rows());
		};
		convertCol(data_entries::AEFF_UM, aeff);
		convertCol(data_entries::CABS_M, cabs);
		convertCol(data_entries::CBK_M, cbk);
		convertCol(data_entries::CEXT_M, cext);
		convertCol(data_entries::CSCA_M, csca);
		convertCol(data_entries::G, g);

		// These are wrong. This just makes the regression and residuals. Need
		// to use lowest to figure out at interpolated and extrapolated points.
		lowess(aeff, cabs, f, nsteps, delta, rcabs, rw, residuals);
		lowess(aeff, cbk, f, nsteps, delta, rcbk, rw, residuals);
		lowess(aeff, cext, f, nsteps, delta, rcext, rw, residuals);
		lowess(aeff, csca, f, nsteps, delta, rcsca, rw, residuals);
		lowess(aeff, g, f, nsteps, delta, rg, rw, residuals);

		res->floatMat = floatMat;
		res->intMat = intMat;
		//res->intMat.resize(intMat.rows(), intMat.cols());

		auto revertCol = [&](int col, const std::vector<double> &in) {
			auto blk = res->floatMat.block(0,col,floatMat.rows(),1);
			for (size_t i=0; i<floatMat.rows(); ++i) {
				blk(i,0) = (float) in[i];
			}
		};
		revertCol(data_entries::CABS_M, rcabs);
		revertCol(data_entries::CBK_M, rcbk);
		revertCol(data_entries::CEXT_M, rcext);
		revertCol(data_entries::CSCA_M, rcsca);
		revertCol(data_entries::G, rg);

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

