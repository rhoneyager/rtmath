#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include "../Ryan_Debug/fs.h"

namespace Ryan_Debug {
	namespace fs {
		template <class T, class U>
		T expandSymlink(const U &p)
		{
			using namespace boost::filesystem;
			// Set a max depth to avoid infinite loops
			const size_t maxDepth = 10;
			size_t d=0;
			path pf(p);
			while(is_symlink(pf) && d<maxDepth)
			{
				pf = boost::filesystem::absolute(read_symlink(pf), pf.parent_path());
				d++;
			}
			return T(pf.string());
		}
		template<class T, class U>
		void expandFolder(const T &pa, 
			std::vector<U> &desta, bool recurse)
		{
			using namespace boost::filesystem;
			path p(pa);
			std::vector<path> dest;
			if (is_directory(p))
			{
				if (!recurse)
					copy(directory_iterator(p), 
					directory_iterator(), back_inserter(dest));
				else
					copy(recursive_directory_iterator(p,symlink_option::recurse), 
					recursive_directory_iterator(), back_inserter(dest));
			}
			else dest.push_back(pa);
			for (const auto & i : dest)
				desta.push_back(U(i.string()));
		}
		template <class T, class U>
		void expandFolder(const std::vector<T> &src, 
			std::vector<U> &dest, bool recurse)
		{
			for (auto s : src)
				expandFolder(s, dest, recurse);
		}

#define EXPAND_SYMLINK(T,U) \
	template T RYAN_DEBUG_DLEXPORT expandSymlink<T, U>(const U &);
#define EXPAND_FOLDER_A(T,U) \
	template void RYAN_DEBUG_DLEXPORT expandFolder(const T &, \
		std::vector<U> &, bool);
#define EXPAND_FOLDER_B(T,U) EXPAND_FOLDER_A(std::vector<T>, U);
#define DOTYPES(f) \
	f(std::string,std::string); \
	f(boost::filesystem::path, boost::filesystem::path); \
	f(std::string, boost::filesystem::path); \
	f(boost::filesystem::path, std::string);

DOTYPES(EXPAND_SYMLINK);
DOTYPES(EXPAND_FOLDER_A);
DOTYPES(EXPAND_FOLDER_B);

	}
}

