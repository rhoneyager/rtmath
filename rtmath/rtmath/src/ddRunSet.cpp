#include <Ryan_Serialization/serialization.h>
#include <boost/serialization/shared_ptr.hpp>
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"

namespace rtmath {
	namespace ddscat {

		dataset::dataset() : id("") {}

		dataset::dataset(const std::string &prefix) : 
			id(prefix) {}

		std::string dataset::getPrefix(const boost::filesystem::path &filename)
		{
			// Prefix is the first part of the filename.
			using namespace std;
			using namespace boost::filesystem;
			path pleaf = filename.filename();
			path proot = filename.parent_path();
			string sleaf = pleaf.string(); // the filename

			size_t pund = sleaf.find('_'); // May be liu or holly
			size_t pshape = sleaf.find("shape"); // Only Holly
			size_t pshp = sleaf.find(".shp"); // Only Liu
			size_t pavg = sleaf.find("avg_"); // Only Liu

			bool isLiu = false, isHolly = false;
			if (pshp != string::npos) isLiu = true;
			if (pavg != string::npos) isLiu = true;
			if ((pshape != string::npos) && !isLiu) isHolly = true;
			if ((pund != string::npos) && !isLiu) isHolly = true;

			// Based on detection criteria, perform appropriate truncation
			if (isLiu)
			{
				// Liu avg files need no manipulation.
				// Remove .shp - pshp for raw shapes and tmatrix results
				if (pshp != string::npos)
					sleaf = sleaf.substr(0,pshp);
				// Raw liu shapes, tmatrix results re now truncated
			}
			if (isHolly)
			{
				size_t pos = 0;
				if ((pund != string::npos) && (pavg != string::npos)) pund = string::npos;
				if (pund == string::npos && pshape != string::npos) pos = pshape;
				else if (pshape == string::npos && pund != string::npos) pos = pund;
				else if (pshape == pund == string::npos) throw;
				else pos = (pund < pshape) ? pund : pshape;
				sleaf = sleaf.substr(0,pos);
			}

			return sleaf;
		}

		bool dataset::isValid(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			if (is_directory(filename)) return false;
			if (isAvg(filename)) return true;
			if (isShape(filename)) return true;
			if (isShapeStats(filename)) return true;
			return false;
		}

		bool dataset::isAvg(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			path pleaf = filename.filename();

			string meth; // replaced with std::string()
			if (Ryan_Serialization::detect_compression(pleaf.string(), meth))
				pleaf.replace_extension();

			path ext = pleaf.extension();

			if (ext.string() == ".avg") return true;
			return false;
		}

		bool dataset::isShape(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			path pleaf = filename.filename();

			string meth; // replaced with std::string()
			if (Ryan_Serialization::detect_compression(pleaf.string(), meth))
				pleaf.replace_extension();

			path ext = pleaf.extension();
			if (ext.string() == ".shp") return true;
			if (pleaf.string().find_last_of("shape.txt") != string::npos) return true; // Lazy check
			return false;
		}

		bool dataset::isShapeStats(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			path pleaf = filename.filename();

			string meth; // replaced with std::string()
			if (Ryan_Serialization::detect_compression(pleaf.string(), meth))
				pleaf.replace_extension();

			path ext = pleaf.extension();
			if (ext.string() == ".xml") return true;
			return false;
		}

		void dataset::prepStats(bool writeStats, const std::string& statsDir)
		{
			if (!shapestatsfile.empty())
			{
				Ryan_Serialization::read(stats,shapestatsfile.string());
				/// \todo Add Ryan_Serialization::read--- T = read<..>(file) alias.
			} else {
				// Generate the stats.
				using namespace std;
				using boost::filesystem::path;
				string statfilename;
				if (writeStats)
				{
					path shapestats;
					shapestats = path(statsDir);
					shapestats /= shapefile.filename();
					shapestats += "-stats.xml";
					shapestatsfile = shapestats.string();
				}
				stats = rtmath::ddscat::shapeFileStats::genStats(
					shapefile.string(), // File to load
					shapestatsfile.string() // Stats file to save
					);
			}
		}

	}
}
