#pragma once
#include "defs.h"
#include <string>

namespace boost
{
	namespace program_options {
		class options_description;
		class variables_map;
	}
	namespace filesystem {
		class path;
	}
}

namespace Ryan_Debug {
	namespace fs {
		/// Commonly-used function that expands any possible symlink in a path.
		/// \param T is input type (std::string or boost::filesystem::path)
		/// \param U is output type (same types as T)
		template <class T, class U>
		T expandSymlink(const U &p);

		/// Commonly-used function that expands a folder's contents.
		/// \param T is input type (std::string or boost::filesystem::path)
		/// \param U is output type (same types as T)
		template <class T, class U>
		void expandFolder(const T &p,
			std::vector<U> &out, bool recurse);
		template <class T, class U>
		void expandFolder(const std::vector<T> &p,
			std::vector<U> &out, bool recurse);

	}
}

