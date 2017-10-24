#include "../Ryan_Debug/info.h"

namespace Ryan_Debug {
	namespace versioning {
		ver_match compareVersions(const versionInfo &a, const versionInfo &b)
		{
			ver_match res = INCOMPATIBLE;

#define tryNum(x) if (a.vn[versionInfo:: x] != b.vn[versionInfo:: x]) return res;
#define tryBool(x) if (a.vb[versionInfo:: x] != b.vb[versionInfo:: x]) return res;
#define tryStr(x) if (std::strncmp(a. x, b. x, versionInfo::charmax ) != 0) return res;
#define tryNumB(x) if ((a.vn[versionInfo:: x] != b.vn[versionInfo:: x]) && a.vn[versionInfo:: x]) return res;
			// First filter the incompatible stuff
			tryStr(vassembly);
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
			tryBool(V_HAS_ZLIB);
			tryBool(V_HAS_GZIP);
			tryBool(V_HAS_BZIP2);
			tryBool(V_HAS_SZIP);

			res = EXACT_MATCH;
			return res;
#undef tryNum
#undef tryBool
#undef tryStr
#undef tryNumB
		}

		void getLibVersionInfo(versionInfo &out)
		{
			genVersionInfo(out);
		}


	}
}