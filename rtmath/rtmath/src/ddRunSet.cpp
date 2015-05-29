#include "Stdafx-ddscat.h"
#include <algorithm>
#include <string>
#include <boost/serialization/shared_ptr.hpp>
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
#include "../../rtmath/rtmath/ddscat/ddpar.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddRunSet.h"
#include <Ryan_Debug/Serialization.h>
#include <Ryan_Debug/error.h>

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
			/// \todo Change to autodetect all of Holly's naming schemes.
			// TODO: Fix here
			if (pund != string::npos)
			{
				size_t pb = sleaf.find_first_not_of("0123456789", pund+1);
				//size_t pb = sleaf.find('_', pund+1);
				if (pb != string::npos) pund = pb;
			}
			size_t pshape = sleaf.find("shape"); // Only Holly
			size_t pshp = sleaf.find(".shp"); // Only Liu
			size_t pavg = sleaf.find("avg_"); // Only Liu

			size_t numUnderscores = std::count(sleaf.begin(), sleaf.end(), '_');

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
				// Newer files follow:
				// ar7_47shape.txt
				// ar7_44_263_36.5.avg (older file naming handled below)
				size_t pos = 0;
				if ((pund != string::npos) && (pavg != string::npos)) pund = string::npos;
				if (pund == string::npos && pshape != string::npos) pos = pshape;
				else if (pshape == string::npos && pund != string::npos) pos = pund;
				else if (pshape == string::npos) RDthrow(Ryan_Debug::error::xBadInput())
					<< Ryan_Debug::error::file_name(filename.string())
					<< Ryan_Debug::error::otherErrorText("Cannot parse filename");
				else if (pund == string::npos) RDthrow(Ryan_Debug::error::xBadInput())
					<< Ryan_Debug::error::file_name(filename.string())
					<< Ryan_Debug::error::otherErrorText("Cannot parse filename");
				else pos = (pund < pshape) ? pund : pshape;
				sleaf = sleaf.substr(0,pos);

				// Holly's original files get a bit of extra manipulation
				if (numUnderscores == 2)
				{
					// 5mm9_combo_shape.txt
					// 5mm9_263_94.0.avg
					sleaf = sleaf.substr(0, sleaf.find_first_of('_'));
				}
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
			if (isPar(filename)) return true;
			return false;
		}

		bool dataset::isAvg(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			path pleaf = filename.filename();

			string meth; // replaced with std::string()
			if (Ryan_Debug::serialization::detect_compression(pleaf.string(), meth))
				pleaf.replace_extension();

			path ext = pleaf.extension();

			if (pleaf.string().find("avg") == 0) return true;
			if (ext.string() == ".avg") return true;
			return false;
		}

		bool dataset::isPar(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			path pleaf = filename.filename();

			string meth; // replaced with std::string()
			if (Ryan_Debug::serialization::detect_compression(pleaf.string(), meth))
				pleaf.replace_extension();

			path ext = pleaf.extension();

			if (ext.string() == ".par") return true;
			return false;
		}

		bool dataset::isShape(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			path pleaf = filename.filename();

			string meth; // replaced with std::string()
			if (Ryan_Debug::serialization::detect_compression(pleaf.string(), meth))
				pleaf.replace_extension();

			path ext = pleaf.extension();
			if (ext.string() == ".shp") return true;
			if (pleaf.string().find("shape.txt") != string::npos) return true; // Lazy check
			return false;
		}

		bool dataset::isShapeStats(const boost::filesystem::path &filename)
		{
			using namespace boost::filesystem;
			using namespace std;
			path pleaf = filename.filename();

			string meth; // replaced with std::string()
			if (Ryan_Debug::serialization::detect_compression(pleaf.string(), meth))
				pleaf.replace_extension();

			path ext = pleaf.extension();
			if (ext.string() == ".xml") return true;
			return false;
		}

		void dataset::prepStats(bool writeStats, const std::string& statsDir)
		{
			if (!shapestatsfile.empty())
			{
				stats = ddscat::stats::shapeFileStats::genStats(shapefile.string(), shapestatsfile.string());
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
				stats = rtmath::ddscat::stats::shapeFileStats::genStats(
					shapefile.string(), // File to load
					shapestatsfile.string() // Stats file to save
					);
			}
		}

	}
}
