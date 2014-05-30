#define RYAN_DEBUG_EXPORTING
#define RYAN_DEBUG_NO_LINK

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include <map>
#include <string>

#ifdef unix
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#endif
#ifdef _WIN32
#include <Windows.h>
#include <Shlwapi.h>
#pragma comment(lib, "Shlwapi")
#endif

#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/modules.h"
#include "internal.h"

namespace Ryan_Debug {

	/// \brief Convenience function to split a null-separated string list into a map of strings.
	///
	/// Commonly used to split up the results of a Ryan_Debug::ProcessInfo environment structure.
	void splitNullMap(
		const std::string &instr, std::map<std::string, std::string> &out)
	{
		using namespace std;
		out.clear();
		if (!instr.size()) return;

		// Fast string splitting based on null values.
		const char* start = instr.data();
		const char* stop = instr.data() + instr.size();
		while (start < stop)
		{
			// Find the next null character
			const char* sep = start;
			sep = std::find(start, stop, '\0');
			if (*start == '\0')
			{
				start = sep + 1;
				continue;
			}
			// Split based on location of equal sign
			//out.push_back(std::string(start, sep - 1));
			const char* sepc = std::find(start, sep, '=');
			// If the = cannot be found, then it is a key with an empty value.
			std::string key(start, sepc);
			if (!key.size())
			{
				start = sep + 1;
				continue;
			}
			/*
			if (//sepc+1 >= stop ||
			//sep > stop ||
			sepc+1>sep)
			{
			std::cerr << "Pointer error: sepc " << &sepc << " sep " << &sep
			<< " stop " << &stop << " start " << &start << std::endl;
			throw;
			}
			*/
			std::string val;
			if (sepc < sep)
				val = std::string(sepc + 1, sep);
			out.insert(std::make_pair(key, val));
			start = sep + 1;
		}
	}

	bool findUserHomeDir(boost::filesystem::path &p)
	{
		// Try to find home directory through env variables first, then via system calls
		boost::shared_ptr<const processInfo> info;
		info = boost::shared_ptr<const processInfo>(getInfo(getPID()), freeProcessInfo);

		size_t sz = 0;
		const char* cEnv = getEnviron(info.get(), sz);
		std::string sEnviron(cEnv, sz);

		std::map<std::string, std::string> mEnv;
		splitNullMap(sEnviron, mEnv);

		if (mEnv.count("HOME"))
		{
			p = boost::filesystem::path(mEnv["HOME"]);
			return true;
		}

		// OS-specific returns
#ifdef __unix__
		struct passwd *pw = getpwuid(getuid());
		const char *homedir = pw->pw_dir;
		p = boost::filesystem::path(std::string(homedir));
		return true;
#endif
#if _WIN32
		if (mEnv.count("USERPROFILE"))
		{
			p = boost::filesystem::path(mEnv["USERPROFILE"]);
			return true;
		}
		return false;
#endif

		return false;
	}

#ifdef _WIN32
	/// Find the base cygwin directory
	bool findCygwinBaseDir(boost::filesystem::path &p)
	{
		// Key is at HKLM\SOFTWARE\Cygwin\setup\rootdir [C:\cygwin64]
		LPCTSTR pszSubKey = "SOFTWARE\\Cygwin\\setup";
		LPCTSTR pszValue = "rootdir";
		DWORD dwType = 0;
		TCHAR sValue[MAX_PATH];
		//LPTSTR sValue = 0;
		DWORD sValueSize = MAX_PATH; //sizeof(DWORD);

		if (ERROR_SUCCESS != SHGetValue(HKEY_LOCAL_MACHINE, pszSubKey, pszValue,
			&dwType, &sValue, &sValueSize))
			return false;

		std::string s = convertStr(sValue);
		p = boost::filesystem::path(s);
		return true;
	}
#endif

	/// Convert a path to the unix equivalent
	boost::filesystem::path convertUnix(const boost::filesystem::path &p)
	{
		using namespace boost::filesystem;
		using std::string;
#ifdef _WIN32
		// If this is a cygwin-relative path, then reexpress the path in the form of 
		// /cygdrive/c/...
		path pabs = boost::filesystem::absolute(p);

		path pres("/cygdrive");
		for (auto it = pabs.begin(); it != pabs.end(); ++it)
		{
			if (it == pabs.begin())
			{
				string sdrv = it->string();
				if (sdrv.size() < 2) return p;
				pres /= path(sdrv.substr(0, 1));
			}
			else {
				if (it->string() == "/") continue;
				pres /= *it;
			}
		}
		return pres;
#else
		return p;
#endif
		return p;
	}

}

