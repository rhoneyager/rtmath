#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include <Ryan_Serialization/serialization.h>

#include "../rtmath/ddscat/ddUtil.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/hash.h"
#include "../rtmath/ddscat/ddpar.h"

namespace rtmath
{
	namespace ddscat
	{
		namespace ddUtil
		{
			void tagTARGETs(const boost::filesystem::path &pBase)
			{
				using namespace boost::filesystem;
				using namespace std;
				path pPar, pShp;
				findDDFiles(pBase,pPar,pShp);

				shapefile shp(pShp.string());
				HASH_t hash = shp.hash();

				string ddver;
				if (!getModuleLoadedDDSCAT(ddver)) ddver = "UNKNOWN";
				

				//using namespace boost::posix_time;
				//ptime now = second_clock::local_time();

				// Iterate through the folder, selecting all sca,
				// fml and avg files. Include compressed entries!
				vector<path> cands;
				copy(recursive_directory_iterator(pBase,symlink_option::no_recurse), 
					recursive_directory_iterator(), back_inserter(cands));
				for (const auto &f : cands)
				{
					if (is_directory(f)) continue;
					string uname, meth, ext;
					Ryan_Serialization::uncompressed_name(f.string(), uname, meth);
					ext = path(uname).extension().string();
					if (ext == ".avg" || ext == ".sca" || ext == ".fml")
						tagTARGET(f,hash,ddver);
				}
			}

			void tagTARGET(const boost::filesystem::path &pFile, 
				HASH_t &hash, const std::string &ddver)
			{
				ddOutputSingle f(pFile.string());
				std::string target;
				f.getTARGET(target);
				// Check if already tagged
				if (target.find("(tag") != std::string::npos) return;

				std::ostringstream out;
				out << "(tag ";

				out << "hash/" << hash.lower;

				out << " ddscat/";
				out << ddver;

				// Set timestamp based on file modification time
				std::time_t lwt = boost::filesystem::last_write_time(pFile);
				boost::posix_time::ptime timestamp = boost::posix_time::from_time_t(lwt);
				out << " time " << timestamp;
				out << " )";

				target = out.str();

				f.setTARGET(target);

				f.writeFile(pFile.string());
				boost::filesystem::last_write_time(pFile,lwt);
			}

			void findDDFiles(const boost::filesystem::path &pStart,
				boost::filesystem::path &pPar,
				boost::filesystem::path &pShp)
			{
				using namespace boost::filesystem;
				path pBase = pStart;
				if (!is_directory(pStart)) pBase.remove_filename();
				
				pPar = pBase / "ddscat.par";

				if (exists(pBase / "target.out")) pShp = pBase / "target.out";
				else if (exists(pBase / "shape.dat")) pShp = pBase / "shape.dat";

			}

			bool getModuleLoadedDDSCAT(std::string &out)
			{
				using namespace std;
				string modstr(getenv("LOADEDMODULES"));
				// string is colon-separated. Expand, then find ddscat/
				typedef boost::tokenizer<boost::char_separator<char> >
					tokenizer;
				boost::char_separator<char> sep(":");
				tokenizer tcom(modstr,sep);
				map<string,string> vals;
				for (auto it=tcom.begin(); it != tcom.end(); ++it)
				{
					boost::char_separator<char> sepb("/");
					tokenizer t(*it,sepb);
					vector<string> params;
					for (auto ot = t.begin(); ot != t.end(); ++ot)
						params.push_back(*ot);
					if (params.size() == 1) vals[params[0]] = "";
					if (params.size() == 2) vals[params[0]] = params[1];
				}
				if (vals.count("ddscat"))
				{
					out = vals["ddscat"];
					return true;
				}
				return false;
			}
		}
	}
}
