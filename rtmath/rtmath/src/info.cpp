#include "Stdafx-core.h"
#include "../rtmath/info.h"

namespace rtmath {
	namespace versioning {
		ver_match compareVersions(const versionInfo &a, const versionInfo &b)
		{
			ver_match res = INCOMPATIBLE;

#define tryNum(x) if (a.vn[versionInfo:: x] != b.vn[versionInfo:: x]) return res;
#define tryStr(x) if (std::strncmp(a. x, b. x, versionInfo::charmax ) != 0) return res;
#define tryNumB(x) if ((a.vn[versionInfo:: x] != b.vn[versionInfo:: x]) && a.vn[versionInfo:: x]) return res;
			// First filter the incompatible stuff
			tryStr(vassembly);
			tryNum(V_MAJOR);
			tryNum(V_MINOR);

			res = COMPATIBLE_1;

			res = COMPATIBLE_2;
			tryNum(V_REVISION);

			res = COMPATIBLE_3;
			tryNum(V_SVNREVISION);

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
