#ifndef SDBR_MAINPP
#define SDBR_MAINPP

#include "defs.hpp"

#include <memory>
#include <string>
#include <Eigen/Dense>

/// This is the main namespace, under which all functions are located.
namespace scatdb_ryan {
	class filter;
	class filterImpl;
	class DLEXPORT_SDBR db {
		friend class filter;
		friend class filterImpl;
		db();
	public:
		~db();
		static std::shared_ptr<const db> loadDB(const char* dbfile = 0);
		static bool findDB(std::string& out);
		void print(std::ostream &out) const;

		// The data in the database, in tabular form
		struct data_entries {
			enum data_entries_floats {
				FREQUENCY_GHZ, TEMPERATURE_K, AEFF_UM, MAX_DIMENSION_MM,
				CABS_M, CBK_M, CEXT_M, CSCA_M, G,
				AS_XY, AS_XZ, AS_YZ,
				NUM_DATA_ENTRIES_FLOATS
			};
			enum data_entries_ints {
				FLAKETYPE,
				NUM_DATA_ENTRIES_INTS
			};
			enum data_entries_stats_fields {
				S_MIN, S_MAX, MEDIAN, MEAN, SD, SKEWNESS, KURTOSIS,
				NUM_DATA_ENTRIES_STATS
			};
			template<class T> static const char* stringify(int enumval);
		};
		typedef Eigen::Matrix<float, Eigen::Dynamic, data_entries::NUM_DATA_ENTRIES_FLOATS> FloatMatType;
		typedef Eigen::Matrix<int, Eigen::Dynamic, data_entries::NUM_DATA_ENTRIES_INTS> IntMatType;
		FloatMatType floatMat;
		IntMatType intMat;

		typedef Eigen::Matrix<float, data_entries::NUM_DATA_ENTRIES_STATS,
			data_entries::NUM_DATA_ENTRIES_FLOATS> StatsFloatType;

		struct data_stats {
			StatsFloatType floatStats;
			int count;
			std::shared_ptr<const db> srcdb;
			void print(std::ostream&) const;
			~data_stats();
			static std::shared_ptr<const data_stats> generate(const db*);
		private:
			data_stats();
		};
		std::shared_ptr<const data_stats> getStats() const;
	};
	class DLEXPORT_SDBR filter {
	private:
		std::shared_ptr<filterImpl> p;
		filter();
	public:
		~filter();
		static std::shared_ptr<filter> generate();
		void addFilterFloat(db::data_entries::data_entries_floats param, float minval, float maxval);
		void addFilterInt(db::data_entries::data_entries_ints param, int minval, int maxval);
		void addFilterFloat(db::data_entries::data_entries_floats param, const std::string &rng);
		void addFilterInt(db::data_entries::data_entries_ints param, const std::string &rng);
		template <class T>
		void addFilter(int param, T minval, T maxval);
		template <class T>
		void addFilter(int param, const std::string &rng);

		//enum class sortDir { ASCENDING, DESCENDING };
		//void addSort(db::data_entries::data_entries_floats param, sortDir);

		std::shared_ptr<const db> apply(std::shared_ptr<const db>) const;
	};
}

#endif

