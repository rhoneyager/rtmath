#ifndef SDBR_MAINPP
#define SDBR_MAINPP

#include "defs.hpp"

#include <memory>
#include <string>
#include <Eigen/Dense>

/// This is the main namespace, under which all functions are located.
namespace scatdb_ryan {
	namespace filters {
	}
	class DLEXPORT_SDBR db {
		db();
	public:
		~db();
		static std::shared_ptr<const db> loadDB(const char* dbfile = 0);
		static bool findDB(std::string& out);

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
		};
		typedef Eigen::Matrix<float, Eigen::Dynamic, data_entries::NUM_DATA_ENTRIES_FLOATS> FloatMatType;
		typedef Eigen::Matrix<int, Eigen::Dynamic, data_entries::NUM_DATA_ENTRIES_INTS> IntMatType;
		FloatMatType floatMat;
		IntMatType intMat;
	};
}

#endif

