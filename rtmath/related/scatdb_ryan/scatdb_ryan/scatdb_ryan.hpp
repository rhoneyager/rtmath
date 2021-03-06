#ifndef SDBR_MAINPP
#define SDBR_MAINPP

#include "defs.hpp"

#include <memory>
#include <string>
#include <Eigen/Dense>

/* TODOs:
 * - add interpolation functions?
 * - add sorting procedures
 * - binned stats with equal flake counts in each bin
 * - c interface
 * - fortran interface
 * - more logging output
 * - ease of use routines for:
 *   - calculating means for all bins
 * - support saving tables of binned results
 */

/// This is the main namespace, under which all functions are located.
namespace scatdb_ryan {
	/// Base class used in pointer marshalling
	class DLEXPORT_SDBR scatdb_base { public: scatdb_base(); virtual ~scatdb_base(); };
	class filter;
	class filterImpl;
	class DLEXPORT_SDBR db : public scatdb_base {
		friend class filter;
		friend class filterImpl;
		db();
	public:
		virtual ~db();
		static std::shared_ptr<const db> loadDB(const char* dbfile = 0);
		static bool findDB(std::string& out);
		void print(std::ostream &out) const;

		// The data in the database, in tabular form
		struct data_entries {
#include "data.h"
			template<class T> static const char* stringify(int enumval);
		};
		typedef Eigen::Matrix<float, Eigen::Dynamic, data_entries::NUM_DATA_ENTRIES_FLOATS> FloatMatType;
		typedef Eigen::Matrix<int, Eigen::Dynamic, data_entries::NUM_DATA_ENTRIES_INTS> IntMatType;
		FloatMatType floatMat;
		IntMatType intMat;

		typedef Eigen::Matrix<float, data_entries::NUM_DATA_ENTRIES_STATS,
			data_entries::NUM_DATA_ENTRIES_FLOATS> StatsFloatType;

		struct data_stats : public scatdb_base {
			StatsFloatType floatStats;
			int count;
			std::shared_ptr<const db> srcdb;
			void print(std::ostream&) const;
			virtual ~data_stats();
			static std::shared_ptr<const data_stats> generate(const db*);
		private:
			data_stats();
		};
		std::shared_ptr<const data_stats> getStats() const;

		/// Regression. See lowess.cpp. delta can equal xrange / 50.
		/// The initial regression routine uses input x values in the output.
		/// For convenience, we re-interpolate over the entire x-value domain,
		/// uniformly spaced in increments of 10 microns. For the lower bound,
		/// interpolation is combined with Rayleigh scattering.
		/// \note This routine should be pre-filtered by temperature,
		///   frequency and flake type.
		std::shared_ptr<const db> regress(
			db::data_entries::data_entries_floats xaxis = db::data_entries::AEFF_UM,
			double f = 0.1,
			long nsteps = 2, double delta = 0.) const;
		std::shared_ptr<const db> interpolate(
			db::data_entries::data_entries_floats xaxis = db::data_entries::AEFF_UM
			) const;
		std::shared_ptr<const db> sort(
			db::data_entries::data_entries_floats xaxis = db::data_entries::AEFF_UM
			) const;
	private:
		mutable std::shared_ptr<const data_stats> pStats;
	};
	class DLEXPORT_SDBR filter : public scatdb_base {
	private:
		std::shared_ptr<filterImpl> p;
		filter();
	public:
		virtual ~filter();
		static std::shared_ptr<filter> generate();
		void addFilterFloat(db::data_entries::data_entries_floats param, float minval, float maxval);
		void addFilterInt(db::data_entries::data_entries_ints param, int minval, int maxval);
		void addFilterFloat(db::data_entries::data_entries_floats param, const std::string &rng);
		void addFilterInt(db::data_entries::data_entries_ints param, const std::string &rng);
		template <class T>
		void addFilter(int param, T minval, T maxval);
		template <class T>
		void addFilter(int param, const std::string &rng);

		void addFilterMaxFlakes(int maxFlakes);
		void addFilterStartingRow(int row);

		enum class sortDir { ASCENDING, DESCENDING };
		void addSortFloat(db::data_entries::data_entries_floats param, sortDir);
		void addSortInt(db::data_entries::data_entries_ints param, sortDir);

		std::shared_ptr<const db> apply(std::shared_ptr<const db>) const;
	};
}

#endif

