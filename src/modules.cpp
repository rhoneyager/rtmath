#define RYAN_DEBUG_EXPORTING
#define RYAN_DEBUG_NO_LINK

#include <boost/filesystem.hpp>
#include <vector>

#ifdef unix
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#endif

#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/modules.h"


namespace Ryan_Debug {

	/// Convert a path to the unix equivalent
	boost::filesystem::path convertUnix(const boost::filesystem::path &p)
	{
		using namespace boost::filesystem;
		using std::string;
#ifdef _WIN32
		// If this is a cygwin-relative path, then reexpress the path in the form of 
		// /cygdrive/c/...
		path pabs = boost::filesystem::absolute(p);
		// root_name
		string sroot = pabs.root_name().string();
		if (sroot.size() < 2) return p;
		string drive(sroot.substr(0,1));
		path pbase = "/cygdrive";
		pbase = pbase / drive;

		
#else
		return p;
#endif
		return p;
	}

}

