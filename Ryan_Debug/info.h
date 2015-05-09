#pragma once
#include <iostream>
#include <sstream>
#include <cstdint>
#include <cstring>

#define QUOTE(str) #str
#define EXPAND_AND_QUOTE(str) QUOTE(str)

namespace Ryan_Debug
{
	namespace versioning {
		struct versionInfo {
			enum nums {
				V_MAJOR, V_MINOR, V_REVISION, V_SVNREVISION,
				V_MSCVER,
				V_GNUC_MAJ, V_GNUC_MIN, V_GNUC_PATCH,
				V_MINGW_MAJ, V_MINGW_MIN,
				V_SUNPRO,
				V_PATHCC_MAJ, V_PATHCC_MIN, V_PATHCC_PATCH,
				V_CLANG_MAJ, V_CLANG_MIN, V_CLANG_PATCH,
				V_INTEL, V_INTEL_DATE,
				V_MAX_INTS
			};
			enum bools {
				V_DEBUG, V_OPENMP, V_AMD64, V_X64, V_UNIX, V_APPLE, V_WIN32,
				V_LLVM,
				V_MAX_BOOLS
			};

			uint64_t vn[V_MAX_INTS];
			bool vb[V_MAX_BOOLS];

			static const uint64_t charmax = 256;
			char vdate[charmax];
			char vtime[charmax];
			char vsdate[charmax];
			char vssource[charmax];
			char vsuuid[charmax];
			char vboost[charmax];
		};

		enum ver_match {
			INCOMPATIBLE, COMPATIBLE_1, COMPATIBLE_2, COMPATIBLE_3, EXACT_MATCH
		};

		// TODO: Embed this into the library!
		ver_match compareVersions(const versionInfo &a, const versionInfo &b)
		{
			ver_match res = INCOMPATIBLE;

#define tryNum(x) if (a.vn[versionInfo:: x] != b.vn[versionInfo:: x]) return res;
#define tryBool(x) if (a.vb[versionInfo:: x] != b.vb[versionInfo:: x]) return res;
#define tryStr(x) if (std::strncmp(a. x, b. x, versionInfo::charmax ) != 0) return res;
#define tryNumB(x) if ((a.vn[versionInfo:: x] != b.vn[versionInfo:: x]) && a.vn[versionInfo:: x]) return res;
			// First filter the incompatible stuff
			tryNum(V_MAJOR);
			tryNum(V_MINOR);
			tryBool(V_AMD64);
			tryBool(V_X64);
			tryBool(V_UNIX);
			tryBool(V_APPLE);
			tryBool(V_WIN32);
			tryStr(vboost);
			tryNum(V_MSCVER);

			res = COMPATIBLE_1;
			tryNumB(V_GNUC_MAJ);
			tryNumB(V_MINGW_MAJ);
			tryNumB(V_SUNPRO);
			tryNumB(V_PATHCC_MAJ);
			tryNumB(V_CLANG_MAJ);
			tryNumB(V_INTEL);

			res = COMPATIBLE_2;
			tryNumB(V_GNUC_MIN);
			tryNumB(V_MINGW_MIN);
			tryNumB(V_PATHCC_MIN);
			tryNumB(V_CLANG_MIN);
			tryNum(V_REVISION);

			res = COMPATIBLE_3;
			tryBool(V_OPENMP);
			tryNum(V_SVNREVISION);

			res = EXACT_MATCH;
			return res;
#undef tryNum
#undef tryBool
#undef tryStr
#undef tryNumB
		}

		/// Calculates Ryan_Debug version string based on compile-time version of external code.
		inline void genVersionInfo(versionInfo &out)
		{
			for (size_t i = 0; i < versionInfo::V_MAX_INTS; ++i) out.vn[i] = 0;
			for (size_t i = 0; i < versionInfo::V_MAX_BOOLS; ++i) out.vb[i] = false;
			out.vsdate[0] = '\0'; out.vssource[0] = '\0'; out.vsuuid[0] = '\0'; out.vboost[0] = '\0';
			std::strncpy(out.vdate, __DATE__, versionInfo::charmax);
			std::strncpy(out.vtime, __TIME__, versionInfo::charmax);

#ifdef __GNUC__
			out.vn[versionInfo::V_GNUC_MAJ] = __GNUC__;
			out.vn[versionInfo::V_GNUC_MIN] = __GNUC_MINOR__;
			out.vn[versionInfo::V_GNUC_PATCH] = __GNUC_PATCHLEVEL__;
#endif
#ifdef __MINGW32__
			out.vn[versionInfo::V_MINGW_MAJ] = __MINGW32_MAJOR_VERSION;
			out.vn[versionInfo::V_MINGW_MIN] = __MINGW32_MINOR_VERSION;
#endif
#ifdef __SUNPRO_CC
			out.vn[versionInfo::V_SUNPRO] = __SUNPRO_CC;
#endif
#ifdef __PATHCC__
			out.vn[versionInfo::V_PATHCC_MAJ] = __PATHCC__;
			out.vn[versionInfo::V_PATHCC_MIN] = __PATHCC_MINOR__;
			out.vn[versionInfo::V_PATHCC_PATCH] = __PATHCC_PATCHLEVEL__;
#endif
#ifdef __llvm__
			out.vb[versionInfo::V_LLVM] = true;
#endif
#ifdef __clang__
			out.vn[versionInfo::V_CLANG_MAJ] = __clang_major__;
			out.vn[versionInfo::V_CLANG_MIN] = __clang_minor__;
			out.vn[versionInfo::V_CLANG_PATCH] = __clang_patchlevel__;
#endif
#ifdef __INTEL_COMPILER
			out.vn[versionInfo::V_INTEL] = __INTEL_COMPILER;
			out.vn[versionInfo::V_INTEL_DATE] = __INTEL_COMPILER_BUILD_DATE;
#endif
#ifdef _MSC_FULL_VER
			out.vn[versionInfo::V_MSCVER] = _MSC_FULL_VER;
#endif
#ifdef BOOST_LIB_VERSION
			strncpy(out.vboost, BOOST_LIB_VERSION, versionInfo::charmax);
#endif
#ifdef SUB_REV
			out.vn[versionInfo::V_SVNREVISION] = SUB_REV;
#endif
#ifdef SUB_DATE
			std::strncpy(out.vsdate, SUB_DATE, versionInfo::charmax);
#endif
#ifdef SUB_SOURCE
			std::strncpy(out.vssource, SUB_SOURCE, versionInfo::charmax);
#endif
#ifdef SUB_UUID
			std::strncpy(out.vsuuid, SUB_UUID, versionInfo::charmax);
#endif
#ifdef _DEBUG
			out.vb[versionInfo::V_DEBUG] = true;
#endif
#ifdef _OPENMP
			out.vb[versionInfo::V_OPENMP] = true;
#endif
#ifdef __amd64
			out.vb[versionInfo::V_AMD64] = true;
#endif
#ifdef _M_X64
			out.vb[versionInfo::V_X64] = true;
#endif
#ifdef __unix__
			out.vb[versionInfo::V_UNIX] = true;
#endif
#ifdef __APPLE__
			out.vb[versionInfo::V_APPLE] = true;
#endif
#ifdef _WIN32
			out.vb[versionInfo::V_WIN32] = true;
#endif
		}

		/**
		* \brief Provides information about the build environment during compilation.
		*
		* This function is designed to provide information on a compiler's
		* build environment. It is a header function because it is designed
		* to reflect the compiler variables of an external project's code.
		*
		* @param out The output stream that receives the information.
		*/
		inline void debug_preamble(const versionInfo &v, std::ostream &out = std::cerr)
		{
			out << "Compiled on " << v.vdate << " at " << v.vtime << std::endl;
			if (v.vssource) out << "SVN Revision " << v.vssource << std::endl;
			if (v.vsdate) out << "SVN Revision Date: " << v.vsdate << std::endl;
			if (v.vssource) out << "SVN Source: " << v.vssource << std::endl;
			if (v.vsuuid) out << "SVN UUID: " << v.vsuuid << std::endl;
			if (v.vb[versionInfo::V_DEBUG]) out << "Debug Version" << std::endl;
			else out << "Release Version" << std::endl;
			if (v.vb[versionInfo::V_OPENMP]) out << "OpenMP enabled in Compiler" << std::endl;
			else out << "OpenMP disabled in Compiler" << std::endl;
			if (v.vb[versionInfo::V_AMD64] || v.vb[versionInfo::V_X64]) out << "64-bit build" << std::endl;
			if (v.vb[versionInfo::V_UNIX]) out << "Unix / Linux Compile" << std::endl;
			if (v.vb[versionInfo::V_APPLE]) out << "Mac Os X Compile" << std::endl;
			if (v.vb[versionInfo::V_WIN32]) out << "Windows Compile" << std::endl;

			if (v.vn[versionInfo::V_GNUC_MAJ])
				out << "GNU Compiler Suite " << v.vn[versionInfo::V_GNUC_MAJ] << "."
				<< v.vn[versionInfo::V_GNUC_MIN] << "." << v.vn[versionInfo::V_GNUC_PATCH] << std::endl;
			if (v.vn[versionInfo::V_MINGW_MAJ])
				out << "MinGW Compiler Suite " << v.vn[versionInfo::V_MINGW_MAJ] << "."
				<< v.vn[versionInfo::V_MINGW_MIN] << std::endl;
			if (v.vn[versionInfo::V_SUNPRO])
				out << "Sun Studio Compiler Suite " << v.vn[versionInfo::V_SUNPRO] << std::endl;
			if (v.vn[versionInfo::V_PATHCC_MAJ])
				out << "EKOPath Compiler " << v.vn[versionInfo::V_PATHCC_MAJ] << "."
				<< v.vn[versionInfo::V_PATHCC_MIN] << "." << v.vn[versionInfo::V_PATHCC_PATCH] << std::endl;
			if (v.vb[versionInfo::V_LLVM]) out << "LLVM Compiler Suite" << std::endl;
			if (v.vn[versionInfo::V_CLANG_MAJ])
				out << "clang compiler " << v.vn[versionInfo::V_CLANG_MAJ] << "."
				<< v.vn[versionInfo::V_CLANG_MIN] << "." << v.vn[versionInfo::V_CLANG_PATCH] << std::endl;
			if (v.vn[versionInfo::V_INTEL])
				out << "Intel Suite " << v.vn[versionInfo::V_INTEL]
				<< " Date " << v.vn[versionInfo::V_INTEL_DATE] << std::endl;
			if (v.vn[versionInfo::V_MSCVER])
				out << "Microsoft Visual Studio Compiler Version " << v.vn[versionInfo::V_MSCVER] << std::endl;
			out << "Boost version " << v.vboost << std::endl;
			out << std::endl;
			out << std::endl;
		}
		inline void debug_preamble(std::ostream &out = std::cerr)
		{
			versionInfo v;
			genVersionInfo(v);
			debug_preamble(v, out);
		}
	}
}

